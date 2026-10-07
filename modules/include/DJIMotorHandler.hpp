#ifndef DJIMOTORHANDLER_HPP
#define DJIMOTORHANDLER_HPP

#include "DJIMotor.hpp"

/**
 * @brief 电机控制类
 */
class DJIMotorHandler
{
public:
    /**
     * 每条 CAN 总线按反馈 ID 0x201~0x208 存放最多 8 个电机实例。
     * 控制帧 0x200 对应槽位 0~3，0x1FF 对应槽位 4~7。
     */
    DJIMotor *DJIMotorList[2][8]{};

    uint8_t can1_send_data_0[8]{}; // CAN1电机控制数据，用于控制0x201-0x204
    uint8_t can1_send_data_1[8]{}; // CAN1电机控制数据，用于控制0x205-0x208

    uint8_t can2_send_data_0[8]{}; // CAN2电机控制数据，用于控制0x201-0x204
    uint8_t can2_send_data_1[8]{}; // CAN2电机控制数据，用于控制0x205-0x208

    // 四个标志位，用于判断是否需要发送控制数据
    bool CAN1_0x200_Exist = false; // CAN1是否存在控制报文为0x200电机
    bool CAN1_0x1FF_Exist = false; // CAN1是否存在控制报文为0x1FF电机
    bool CAN2_0x200_Exist = false; // CAN2是否存在控制报文为0x200电机
    bool CAN2_0x1FF_Exist = false; // CAN2是否存在控制报文为0x1FF电机

    /**
     * @brief 注册电机并绑定其 CAN 句柄和反馈 ID。
     * @param DJImotor 电机实例，须在注册后保持有效。
     * @param hcan CAN1 或 CAN2 句柄。
     * @param canId 反馈标准 ID，支持 0x201~0x208。
     */
    void registerMotor(DJIMotor *DJImotor, CAN_HandleTypeDef *hcan, uint16_t canId); // 使用指针作为参数

    /**
     * @brief 为已注册电机计算输出并发送分组控制帧。
     * @return 任一已注册分组的发送均成功时返回 true。
     */
    bool sendControlData();

    /**
     * @brief 处理并更新电机反馈数据
     * @param hcan CAN 句柄
     * @param rx_data 反馈数据
     * @param index 反馈 ID 偏移，范围 0~7（对应 0x201~0x208）。
     */
    void updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index);

    /** 检查全部已注册电机在上一检查周期内是否收到新反馈。 */
    void AllMotorAliveCheck();

    /** 解析一帧电调反馈，更新编码器、转速、电流、温度和累计位置。 */
    void UpdateSensorData(DJIMotor *motor, uint8_t *can_receive_data);

    /** 返回进程内唯一的电机注册表及 CAN 控制帧管理器。 */
    static DJIMotorHandler *Instance()
    {
        static DJIMotorHandler instance;
        return &instance;
    }
};

#endif // DJIMOTORHANDLER_HPP
