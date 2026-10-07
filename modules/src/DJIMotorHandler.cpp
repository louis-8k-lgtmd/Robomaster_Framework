#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"
#include "math.hpp"

namespace
{
/*
 * 本文件的内部协议辅助函数，不暴露到公共头文件：
 * CAN 句柄映射为管理器的 bus 维度；电调反馈多字节字段按大端解析；
 * 转子反馈速度按电机型号转换到输出轴速度时使用对应减速比。
 */
// 将 HAL 句柄映射到 DJIMotorList 的总线维度，未知总线返回 -1。
int GetCanIndex(const CAN_HandleTypeDef *hcan)
{
    if (hcan == nullptr)
        return -1;
    if (hcan->Instance == CAN1)
        return 0;
    if (hcan->Instance == CAN2)
        return 1;
    return -1;
}

/*
 * 读取两个大端字节并解释为有符号二补码 int16。
 * 先按无符号组合，再对最高位为 1 的值减 2^16，避免依赖主机端序和
 * 超出 int16_t 范围的隐式窄化行为。
 */
int16_t ReadInt16BE(const uint8_t *data)
{
    const uint16_t value = (static_cast<uint16_t>(data[0]) << 8U) | data[1];
    return static_cast<int16_t>(value < 0x8000U ? value : static_cast<int32_t>(value) - 0x10000);
}

/*
 * DJI 电调上报转子速度（rpm），控制器使用输出轴物理量。
 * 这里返回电机型号减速比；未知/无减速箱按 1:1 处理。
 */
float GetGearRatio(GearBox gearBox)
{
    switch (gearBox)
    {
    case GearBox_M2006:
        return 36.0f;
    case GearBox_M3508:
        return 19.0f;
    default:
        return 1.0f;
    }
}
}

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    /*
     * 注册流程：验证电机指针、总线及反馈 ID -> 检查槽位冲突和重复实例 ->
     * 保存映射 -> 标记对应控制帧分组存在。
     * 每条总线 0x201~0x208 映射到索引 0~7；本函数不动态分配电机对象，
     * 调用者必须保证 motor 对象在注册期间及之后持续有效。
     */
    // 只接受反馈 ID 0x201~0x208；数组槽位由反馈 ID 减 0x201 得到。
    const int canIndex = GetCanIndex(hcan);
    if (motor == nullptr || canIndex < 0 || canId < 0x201U || canId > 0x208U)
        return;

    const int motorIndex = canId - 0x201U;
    // 避免覆盖其他电机，也禁止同一实例同时占用两个 CAN 槽位。
    if (DJIMotorList[canIndex][motorIndex] != nullptr &&
        DJIMotorList[canIndex][motorIndex] != motor)
        return;

    for (int bus = 0; bus < 2; ++bus)
    {
        for (int slot = 0; slot < 8; ++slot)
        {
            // 一个 motor 实例只能注册在一个 bus/slot，防止同一对象被重复控制。
            if (DJIMotorList[bus][slot] == motor && (bus != canIndex || slot != motorIndex))
                return;
        }
    }

    // 保存总线身份供反馈路由校验；输出初始化为 0，确保首次控制帧不带随机电流。
    DJIMotorList[canIndex][motorIndex] = motor;
    motor->hcan = hcan;
    motor->canId = canId;
    motor->currentSet = 0;

    // DJI 将每路 CAN 的 8 台电机分成两个控制报文，每个报文承载 4 个 int16 电流。
    if (canIndex == 0)
    {
        if (motorIndex < 4)
            CAN1_0x200_Exist = true;
        else
            CAN1_0x1FF_Exist = true;
    }
    else if (motorIndex < 4)
        CAN2_0x200_Exist = true;
    else
        CAN2_0x1FF_Exist = true;
}

bool DJIMotorHandler::sendControlData()
{
    /*
     * 发送阶段的数据流：
     *   遍历 CAN1/CAN2 和两个 ID 分组 -> 调用已注册电机的 setOutput() 计算电流 ->
     *   将 4 个有符号电流按大端顺序写入 8 字节帧 -> 交由 CAN BSP 发送。
     * 第一组索引 0~3 使用报文 ID 0x200，第二组索引 4~7 使用 0x1FF。
     */
    // 二维表将 bus/group 索引直接映射到对应的持久帧缓冲区。
    uint8_t *frames[2][2] = {
        {can1_send_data_0, can1_send_data_1},
        {can2_send_data_0, can2_send_data_1}};
    const bool send[2][2] = {
        {CAN1_0x200_Exist, CAN1_0x1FF_Exist},
        {CAN2_0x200_Exist, CAN2_0x1FF_Exist}};

    // 只发送含已注册电机的分组；未注册槽输出零，避免残留上次帧数据。
    bool sent = false;
    for (int bus = 0; bus < 2; ++bus)
    {
        for (int group = 0; group < 2; ++group)
        {
            if (!send[bus][group])
                continue;

            uint8_t *frame = frames[bus][group];
            for (int slot = 0; slot < 4; ++slot)
            {
                // 一个 CAN 控制帧中该槽对应的电机实例；空槽保持零电流。
                DJIMotor *motor = DJIMotorList[bus][group * 4 + slot];
                if (motor != nullptr)
                    // 各电机派生类根据控制模式和反馈更新 currentSet。
                    motor->setOutput();
                const int16_t current = motor == nullptr ? 0 : motor->currentSet;
                const uint16_t value = static_cast<uint16_t>(current);
                // 保留 int16_t 的二补码位模式，并按 CAN 协议的大端字节序打包。
                frame[slot * 2] = static_cast<uint8_t>(value >> 8U);
                frame[slot * 2 + 1] = static_cast<uint8_t>(value);
            }

            const uint32_t frameId = group == 0 ? 0x200U : 0x1FFU;
            // 任一必要报文发送失败即返回失败；后续分组不再尝试本轮发送。
            if (!CAN_Transmit(bus == 0 ? &hcan1 : &hcan2, frameId, frame, 8))
                return false;
            sent = true;
        }
    }
    return sent;
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    /*
     * 这是 CAN BSP 接收回调进入电机管理器的路由入口。
     * index 应为反馈 ID - 0x201；校验总线、数据指针和数组范围后，
     * 只有槽位中已注册且总线句柄完全匹配的实例才接收该帧。
     */
    const int canIndex = GetCanIndex(hcan);
    if (canIndex < 0 || rx_data == nullptr || index < 0 || index >= 8)
        return;

    DJIMotor *motor = DJIMotorList[canIndex][index];
    if (motor != nullptr && motor->hcan == hcan)
        UpdateSensorData(motor, rx_data);
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    if (motor == nullptr || can_data == nullptr)
        return;

    auto &feedback = motor->motorFeedback;
    /*
     * DJI 反馈帧固定 8 字节布局：
     * [0..1] 编码器计数(0..8191，无符号大端)，[2..3] 转子速度 rpm，
     * [4..5] 电流反馈，[6] 温度；最后一个字节目前不参与控制逻辑。
     */
    const uint16_t ecd = static_cast<uint16_t>((static_cast<uint16_t>(can_data[0]) << 8U) | can_data[1]);
    const int16_t speed = ReadInt16BE(&can_data[2]);
    const float ratio = GetGearRatio(motor->gearBox);

    // 更新前保存上一拍状态，供上层需要时计算速度/位置变化。
    feedback.lastPositionFdb = feedback.positionFdb;
    feedback.lastSpeedFdb = feedback.speedFdb;
    feedback.last_ecd = static_cast<int16_t>(feedback.ecd);
    feedback.ecd = ecd;
    feedback.speed_rpm = speed;
    // 解码反馈中的转速、电流和温度；编码器本身按无符号计数处理。
    feedback.currentFdb = ReadInt16BE(&can_data[4]);
    feedback.temperatureFdb = can_data[6];
    // rpm -> 转子 rad/s -> 除以减速比得到输出轴 rad/s。
    feedback.speedFdb = speed * (6.28318530718f / 60.0f / ratio);

    if (feedback.ecdInitialized)
    {
        /*
         * 编码器为单圈 13 位计数，跨越 8191->0 或 0->8191 时原始差值会接近整圈。
         * 大于半圈的差值按负向回绕修正，小于负半圈的差值按正向回绕修正；
         * 因而假设相邻两帧之间真实转动小于半圈，才能唯一确定方向。
         */
        int32_t delta = static_cast<int32_t>(ecd) - feedback.last_ecd;
        if (delta > 4095)
            delta -= 8192;
        else if (delta < -4095)
            delta += 8192;
        // 计数增量换算为输出轴弧度并累加；位置以首帧之后的相对变化为基准。
        feedback.positionFdb += delta * (6.28318530718f / 8192.0f / ratio);
    }
    else
    {
        // 首帧只建立计数基准，不把未知上电相位解释成位置变化。
        feedback.ecdInitialized = true;
    }

    // 每收到一帧有效路由后的反馈递增；AliveCheck 据此判断检查窗口内是否在线。
    ++motor->AliveFlag;
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    // 遍历两条 CAN 总线的注册表，只调用有效实例的具体在线状态检查实现。
    for (auto &bus : DJIMotorList)
    {
        for (DJIMotor *motor : bus)
        {
            if (motor != nullptr)
                motor->AliveCheck();
        }
    }

}

extern "C" uint8_t MotorControlTaskStep(void)
{
    /*
     * C ABI 周期任务入口，由 app_threadx.c 的 Motor Control 线程调用。
     * 每次先计算并发送当前电机命令，再用调用次数分频执行在线检查；
     * 当前线程约每 10 ms 调用一次，因此 10 次约为 100 ms 检查窗口。
     */
    static uint8_t alive_check_divider;
    DJIMotorHandler *handler = DJIMotorHandler::Instance();
    const bool sent = handler->sendControlData();
    if (++alive_check_divider >= 10U)
    {
        // 该计数依据控制任务周期换算；调整线程周期时也需相应更新分频值。
        handler->AllMotorAliveCheck();
        alive_check_divider = 0U;
    }
    return sent ? 1U : 0U;
}
