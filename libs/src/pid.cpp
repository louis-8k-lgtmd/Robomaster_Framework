#include "pid.hpp"

/*
 * 离散 PID 控制器实现。
 *
 * UpdateResult 每被调用一次代表控制算法进行一个采样周期；本实现不接收 dt，
 * 因此 kp/ki/kd 应按调用周期及 ref/fdb 的单位整定。误差定义为 ref-fdb，
 * 正增益下正误差产生正向控制量。位置式和增量式共享同一组状态变量，
 * 不应在运行中随意切换 mode，否则 result/iResult 的状态意义会发生变化。
 */
PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    // 初始化动态状态；Clear 不改写本构造函数刚设置的增益、模式和限幅。
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    /*
     * 在线更新控制增益。保留积分项、输出及误差历史，避免调参时状态突变；
     * 因此新旧增益交替作用时，调用者仍需确认当前积分状态与新参数相容。
     */
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    /*
     * 首先滚动保存误差历史：
     * err[0] = e[k]，err[1] = e[k-1]，err[2] = e[k-2]。
     * 导数项的离散差分公式依赖这些历史值；首次计算时历史由 Clear() 置零。
     */
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    switch (mode)
    {
    case PID_POSITION:
        /*
         * 位置式 PID 直接计算本周期完整输出：
         *   P[k] = Kp * e[k]
         *   I[k] = clamp(I[k-1] + Ki * e[k], -maxIOut, +maxIOut)
         *   D[k] = Kd * (e[k] - e[k-1])
         *   u[k] = clamp(P[k] + I[k] + D[k], -maxOut, +maxOut)
         *
         * 这里的“位置式”指 result 是绝对控制输出，而不是位置传感器的单位。
         * 微分项没有除以采样周期，积分项也没有乘显式 dt，周期影响由参数整定吸收。
         */
        pResult = kp * err[0];
        // maxIOut<=0 时传入零限幅，因此积分状态被限制为 0（禁用积分累积）。
        iResult = Numeric::LimitABS(iResult + ki * err[0], maxIOut > 0.0f ? maxIOut : 0.0f);
        // 对误差做一阶差分；恒定误差时该项为零。
        dResult = kd * (err[0] - err[1]);
        // 即使各分项未超限，三项之和仍可能过大，故对总输出再做一次限幅。
        result = Numeric::LimitABS(pResult + iResult + dResult, maxOut > 0.0f ? maxOut : 0.0f);
        break;

    case PID_DELTA:
    {
        /*
         * 增量式 PID 计算输出变化量，再叠加到上一拍输出：
         *   ΔP = Kp * (e[k] - e[k-1])
         *   ΔI = I[k] - I[k-1] = Ki * e[k]（受积分限幅影响）
         *   ΔD = Kd * (e[k] - 2e[k-1] + e[k-2])
         *   u[k] = clamp(u[k-1] + ΔP + ΔI + ΔD, -maxOut, +maxOut)
         *
         * previousI 用于将本拍积分状态变化量而非积分总值加到输出上；
         * result 则承载上一拍累计输出，因此增量式必须保留 result 状态。
         */
        pResult = kp * (err[0] - err[1]);
        const float previousI = iResult;
        // 更新受限积分状态；其变化量在下方通过 iResult-previousI 加入本拍输出。
        iResult = Numeric::LimitABS(iResult + ki * err[0], maxIOut > 0.0f ? maxIOut : 0.0f);
        // 二阶误差差分近似“导数的变化”，构成增量式的微分输出项。
        dResult = kd * (err[0] - 2.0f * err[1] + err[2]);
        // 累加本拍三个增量，并限制累计输出；maxOut<=0 时输出被限制为零。
        result = Numeric::LimitABS(result + pResult + iResult - previousI + dResult,
                                   maxOut > 0.0f ? maxOut : 0.0f);
        break;
    }

    default:
        // 未识别模式不运行控制计算，清零各分项和结果，避免沿用旧输出。
        pResult = iResult = dResult = result = 0.0f;
        break;
    }
}

void PID::Clear()
{
    /*
     * 清除控制器的动态状态：
     * ref/fdb、误差历史、P/I/D 分项和最终输出全部归零。
     * 增益 kp/ki/kd、模式 mode、输出限幅 maxOut/maxIOut 保持不变，
     * 便于停机后复位状态并继续沿用同一套参数。
     */
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
