#include "control_task.h"
#include "DJIMotorHandler.hpp"
#include "M2006.hpp"
#include "can.h"
#include "tx_api.h"

#include <cmath>

namespace
{
/*
 * 本控制任务示例管理一台 M2006。
 * 电机对象具有静态存储期：DJIMotorHandler 仅保存其地址而不拥有/复制对象，
 * 因此对象在整个程序运行期间有效，不会因初始化函数返回而悬空。
 */
M2006 motor;
}

extern "C" uint8_t ControlTask_Init(void)
{
    /*
     * 将电机绑定到 CAN1 反馈 ID 0x201。
     * 管理器按反馈 ID 确定数组槽位；0x201 对应 CAN1[0][0]，
     * 其控制命令随后放在 CAN 控制帧 0x200 的第一个 16 位字段中。
     * 注册只建立软件关联，不会启用闭环或自动输出目标。
     */
    DJIMotorHandler *handler = DJIMotorHandler::Instance();
    handler->registerMotor(&motor, &hcan1, 0x201U);
    // registerMotor 以拒绝无效参数/冲突注册的方式返回；用注册表确认本次绑定结果。
    return handler->DJIMotorList[0][0] == &motor ? 1U : 0U;
}

extern "C" uint8_t ControlTask_SetPositionTarget(float position_rad)
{
    /*
     * 上层显式请求进入位置控制。目标单位为电机输出轴弧度，
     * 位置反馈由注册电机收到的首帧编码器数据开始累计，因此在首帧前没有
     * 可比较的位置参考，必须拒绝设置，避免用未初始化位置驱动电机。
     */
    // 非有限目标会使 PID 计算产生无效数值，故在写入共享状态前拒绝。
    if (!std::isfinite(position_rad))
        return 0U;

    /*
     * CAN 接收中断会更新编码器反馈，控制线程会读取 controlMode/positionSet。
     * 短暂关闭中断，将反馈就绪检查、目标写入和模式切换作为一个原子状态更新。
     */
    const UINT interrupt_state = tx_interrupt_control(TX_INT_DISABLE);
    if (!motor.motorFeedback.ecdInitialized)
    {
        tx_interrupt_control(interrupt_state);
        return 0U;
    }
    // 先写目标，再切换到 POS_MODE，使控制线程看到模式时目标已准备好。
    motor.positionSet = position_rad;
    motor.controlMode = DJIMotor::POS_MODE;
    tx_interrupt_control(interrupt_state);
    return 1U;
}

extern "C" void ControlTask_Stop(void)
{
    /*
     * 安全停机：将控制模式退回 RELAX，并立即把待发电流命令置零。
     * 临界区避免 CAN 控制线程在 mode 与 currentSet 更新之间读取到不一致状态；
     * 下一个控制周期仍会由 M2006::setOutput() 清积分并保持零输出。
     */
    const UINT interrupt_state = tx_interrupt_control(TX_INT_DISABLE);
    motor.controlMode = DJIMotor::RELAX_MODE;
    motor.currentSet = 0;
    tx_interrupt_control(interrupt_state);
}
