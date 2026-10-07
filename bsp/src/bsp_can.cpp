#include "bsp_can.hpp"
#include "DJIMotorHandler.hpp"

/*
 * CAN BSP 在此连接 HAL 与上层电机管理器：
 *   启动阶段：CAN_Init 配置过滤器、启动 CAN1/CAN2 并打开 FIFO0 通知；
 *   发送方向：电机管理器组帧 -> CAN_Transmit -> HAL 发送邮箱；
 *   接收方向：CAN FIFO0 中断 -> 本文件回调取帧/筛选 -> 电机管理器更新反馈。
 *
 * 本文件只处理 CAN 外设和帧传递，不负责电机 PID 或协议以外的业务。
 */
namespace
{
// 仅允许使用 CubeMX 创建且已配置的 CAN1/CAN2 句柄，避免把无效句柄交给 HAL。
bool IsValidCan(CAN_HandleTypeDef *hcan)
{
    return hcan != nullptr && (hcan->Instance == CAN1 || hcan->Instance == CAN2);
}
}

/**
 * @brief 完成两路 CAN 控制器的接收配置和启动。
 *
 * 配置顺序为：设置共享过滤器 bank 分界 -> 配置/启动 CAN1 -> 配置/启动 CAN2；
 * 每路控制器启动后都启用 FIFO0“消息待处理”通知，以便 HAL 在接收中断中调用下方回调。
 * FilterId 和 FilterMask 保持为零，因此 ID-mask 过滤器暂时放行所有帧，
 * 有效帧类型与电机反馈 ID 由接收回调的软件筛选。
 *
 * @return 两路过滤器、控制器和 FIFO0 通知均成功时返回 1，否则返回 0。
 * @note STM32 双 CAN 共用过滤器组：CAN1 使用 bank 0~13，CAN2 从 bank 14 开始。
 */
uint8_t CAN_Init(void)
{
    // 零初始化可确保过滤器 ID/mask 字段为零；FIFO0 是本模块唯一读取的接收 FIFO。
    CAN_FilterTypeDef filter = {};
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0;
    filter.SlaveStartFilterBank = 14;

    // 用短路判断确保任一阶段失败立即返回；失败时上层可将 CAN 标记为未就绪。
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return 0U;

    // CAN2 使用从属 bank 区域，避免覆盖 CAN1 的过滤器配置。
    filter.FilterBank = 14;
    return HAL_CAN_ConfigFilter(&hcan2, &filter) == HAL_OK &&
                   HAL_CAN_Start(&hcan2) == HAL_OK &&
                   HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK
               ? 1U
               : 0U;
}

uint8_t CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    /*
     * 先做调用参数及硬件状态检查：
     * - 只允许 CAN1/CAN2 及标准帧 ID；
     * - Classic CAN 数据字段最多 8 字节；
     * - 控制器必须已启动并处于 LISTENING 状态，且至少有一个发送邮箱空闲。
     * 失败通过返回值报告，不在常见的暂时性邮箱拥塞场景调用 Error_Handler。
     */
    if (!IsValidCan(hcan) || msg == nullptr || Id > 0x7FFU || len > 8U ||
        HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING ||
        HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return 0U;
    }

    // 显式指定标准数据帧；调用者传入的 Id 和 len 分别成为 StdId 与 DLC。
    CAN_TxHeaderTypeDef header = {};
    header.StdId = Id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;
    uint32_t mailbox;
    /*
     * HAL 将发送请求放入可用硬件邮箱，mailbox 接收被分配的邮箱编号。
     * HAL_OK 表示请求已被 CAN 外设接受，不代表总线上已经成功发送或收到 ACK；
     * 后续发送完成/错误状态由 CAN 硬件和 HAL 状态机制反映。
     */
    return HAL_CAN_AddTxMessage(hcan, &header, msg, &mailbox) == HAL_OK ? 1U : 0U;
}

/**
 * @brief HAL 的 FIFO0 接收中断回调，将电机反馈帧转交给电机管理器。
 *
 * 一个中断可能对应 FIFO0 中多帧积压，因此循环取出直至 FIFO 为空。
 * 每帧先由 HAL 读取并释放 FIFO 项，再筛选标准数据帧、8 字节 DLC 和
 * 电机反馈 ID 0x201~0x208；其余帧已从 FIFO 取走但不进入电机反馈处理。
 * 电机索引按 ID 偏移换算：0x201 -> 0，...，0x208 -> 7。
 *
 * @note 此函数运行在中断上下文；被调用的反馈更新逻辑应保持短小、不可阻塞，
 *       不应在其中执行等待、打印或耗时的控制计算。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 对不属于本板 CAN1/CAN2 的回调不做处理。
    if (!IsValidCan(hcan))
    {
        return;
    }

    // 排空当前 FIFO0，避免仍有帧待处理时丢失后续通知或造成积压。
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        // 读取头部和最多 8 字节数据，同时由 HAL 从 FIFO0 移除该帧。
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            // FIFO 读取失败表示 HAL/外设异常，沿用板级统一的致命错误处理。
            Error_Handler();
            return;
        }

        // 仅 DJI 电机反馈格式进入电机层；扩展帧、远程帧及其他 ID 在 BSP 层过滤。
        if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA &&
            header.DLC == 8U && header.StdId >= 0x201U && header.StdId <= 0x208U)
        {
            // 传入接收总线和 0~7 槽位，管理器据此找到对应注册电机并解析反馈。
            DJIMotorHandler::Instance()->updateFeedback(hcan, data, static_cast<int>(header.StdId - 0x201U));
        }
    }
}
