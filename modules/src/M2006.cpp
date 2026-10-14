#include "M2006.hpp"
#include "math.hpp"

#include <cmath>
#include <limits>

namespace
{
/* 调参入口：输出轴单位 rad、rad/s；PID 每 10 ms 更新一次。 */
constexpr float kSpeedKp = 225.0f;
constexpr float kSpeedKi = 2.5f;
constexpr float kSpeedKd = 1.0f;
constexpr float kSpeedIntegralLimit = 600.0f;
constexpr float kPositionKp = 10.0f;
constexpr float kPositionKi = 0.0f;
constexpr float kPositionKd = 0.0f;
constexpr float kPositionMaxSpeedRadS = 3.0f;
constexpr uint16_t kCurrentCommandLimit = 2000U;
/* Small, smooth current bias helps the motor cross low-speed static friction. */
constexpr float kFrictionCurrent = 100.0f;
constexpr float kFrictionBlendSpeedRadS = 0.15f;
/* Feed forward a bounded torque at sine-wave reversals (C610 current units). */
constexpr float kAccelerationCurrentPerRadS2 = 250.0f;
/* Supply part of the running torque without requiring a steady speed error. */
constexpr float kVelocityCurrentPerRadS = 120.0f;
/* Extra current while stationary; fade it out once the rotor starts moving. */
constexpr float kBreakawayCurrent = 200.0f;
constexpr float kBreakawayDemandRadS = 0.10f;
constexpr float kBreakawayFadeSpeedRadS = 0.15f;
constexpr float kBreakawayAccelerationLookaheadS = 0.40f;
constexpr float kReversalDemandThresholdRadS = 0.05f;
}

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
    maxCurrent = kCurrentCommandLimit;
    speedPid = PID(kSpeedKp, kSpeedKi, kSpeedKd, maxCurrent,
                   kSpeedIntegralLimit, PID_POSITION);
    positionPid = PID(kPositionKp, kPositionKi, kPositionKd,
                      kPositionMaxSpeedRadS, 0.0f, PID_POSITION);
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
        positionDemandDirection = 0;
        speedPid.ref = speedSet;
        break;
    case POS_MODE:
        // 闭环位置控制使用累计输出轴角度作为反馈，目标和反馈均为 rad。
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();
        // Add the sine reference derivative so the position loop corrects tracking error.
        speedPid.ref = Numeric::LimitABS(
            positionPid.result + positionFeedforwardRadS, positionPid.maxOut);
        if (std::isfinite(speedPid.ref))
        {
            const int8_t direction = speedPid.ref > kReversalDemandThresholdRadS ? 1 :
                                     speedPid.ref < -kReversalDemandThresholdRadS ? -1 : 0;
            if (direction != 0)
            {
                if (positionDemandDirection != 0 && direction != positionDemandDirection)
                    speedPid.iResult = 0.0f;
                positionDemandDirection = direction;
            }
        }
        break;
    case RELAX_MODE:
    default:
        // 安全松开时清空两级 PID 的积分、误差历史和输出，避免旧状态导致重新使能突跳。
        speedPid.Clear();
        positionPid.Clear();
        positionDemandDirection = 0;
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
    float output = speedPid.result;
    if (controlMode == POS_MODE && std::isfinite(speedPid.ref))
    {
        const float command_speed = speedPid.ref;
        const float command_acceleration =
            std::isfinite(positionAccelerationFeedforwardRadS2)
                ? positionAccelerationFeedforwardRadS2 : 0.0f;
        output += kFrictionCurrent * command_speed /
                  (std::fabs(command_speed) + kFrictionBlendSpeedRadS);
        if (std::isfinite(positionFeedforwardRadS))
            output += kVelocityCurrentPerRadS * positionFeedforwardRadS;
        output += kAccelerationCurrentPerRadS2 * command_acceleration;
        const float actual_speed = std::fabs(speedPid.fdb);
        if (actual_speed < kBreakawayFadeSpeedRadS)
        {
            const float fade = 1.0f - actual_speed / kBreakawayFadeSpeedRadS;
            const float reversal_demand = command_speed +
                kBreakawayAccelerationLookaheadS * command_acceleration;
            output += kBreakawayCurrent * fade * reversal_demand /
                      (std::fabs(reversal_demand) + kBreakawayDemandRadS);
        }
    }
    output = Numeric::LimitABS(output, currentLimit);
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
