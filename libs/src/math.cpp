#include "math.hpp"

namespace Numeric
{
    /*
     * 对称限幅（clamp）：
     *   - 输入大于上限时返回 +maxValue；
     *   - 输入小于下限时返回 -maxValue；
     *   - 输入位于闭区间 [-maxValue, +maxValue] 内时原样返回。
     *
     * maxValue 是“幅值上限”而非带符号的上下界，因此调用方须传入非负值。
     * PID 用它限制积分项和最终输出，电机控制也用它约束电流命令。
     * 本函数仅比较并返回边界，不负责修改输入变量，也不检查 NaN/无穷值。
     * maxValue 为 0 时，普通有限输入会被限制为 0。
     */
    float LimitABS(float input, float maxValue)
    {
        // 大于正向边界时饱和到正向边界；等于边界时保留原输入。
        if (input > maxValue)
            return maxValue;
        // 小于负向边界时饱和到负向边界。
        if (input < -maxValue)
            return -maxValue;
        // 未超限时不做缩放，保持输入值和符号不变。
        return input;
    }

}
