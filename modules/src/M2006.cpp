#include "M2006.hpp"
#include "math.hpp"

#include <cmath>
#include <limits>

M2006::M2006()
{
    /*
     * M2006 控制器初始参数：
     * - 减速比 36:1，用于反馈速度/位置从转子侧换算到输出轴；
     * - maxCurrent 为发给电调的电流命令限幅；
     * - speedPid 是内环，输入/反馈为输出轴角速度，输出为电流命令；
     * - positionPid 是外环，位置误差转换为内环角速度目标。
     * PID 增益是工程初值，必须按实际控制周期、机械负载和硬件方向整定。
     * DJIMotor 构造时默认 RELAX_MODE，因此上电不会自动驱动电机。
     */
    gearBox = GearBox_M2006;
    maxCurrent = 10000;
    speedPid = PID(100.0f, 0.1f, 0.0f, maxCurrent, 2000.0f, PID_POSITION);
    positionPid = PID(8.0f, 0.0f, 0.0f, 20.0f, 0.0f, PID_POSITION);
}

void M2006::setOutput()
{
    /*
     * 将当前模式的目标转换为电流命令：
     * SPD_MODE 直接使用 speedSet 作为速度内环目标；
     * POS_MODE 先由位置外环生成速度目标，再统一进入速度内环；
     * RELAX_MODE 清除控制器记忆状态并保持零电流。
     */
    switch (controlMode)
    {
    case SPD_MODE:
        // 纯速度模式跳过位置环，外部 speedSet 单位为输出轴 rad/s。
        speedPid.ref = speedSet;
        break;
    case POS_MODE:
        // 闭环位置控制使用累计输出轴角度作为反馈，目标和反馈均为 rad。
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();
        // 位置 PID 的输出定义为速度目标，受 positionPid.maxOut 限制。
        speedPid.ref = positionPid.result;
        break;
    case RELAX_MODE:
    default:
        // 安全松开时清空两级 PID 的积分、误差历史和输出，避免旧状态导致重新使能突跳。
        speedPid.Clear();
        positionPid.Clear();
        currentSet = 0;
        return;
    }

    // 两种闭环模式最终都由速度内环比较目标与当前输出轴速度。
    speedPid.fdb = motorFeedback.speedFdb;
    speedPid.UpdateResult();
    /*
     * PID 输出先按 maxCurrent 与 CAN int16 字段上限取较小值对称限幅，
     * 再转换成整数电流命令。非有限值不允许进入 CAN 帧，故回退为零。
     */
    const float currentLimit = maxCurrent < std::numeric_limits<int16_t>::max()
                                   ? maxCurrent
                                   : std::numeric_limits<int16_t>::max();
    const float output = Numeric::LimitABS(speedPid.result, currentLimit);
    currentSet = std::isfinite(output) ? static_cast<int16_t>(output) : 0;
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    /*
     * 由管理器周期调用。若 AliveFlag 在本检查窗口内增加，说明至少收到过反馈；
     * 更新 Pre_AliveFlag 作为下一窗口基准。连续一个窗口无新反馈即判离线。
     */
    MotorState = AliveFlag != Pre_AliveFlag ? MOTOR_ONLINE : MOTOR_OFFLINE;
    Pre_AliveFlag = AliveFlag;
    return MotorState;
}
