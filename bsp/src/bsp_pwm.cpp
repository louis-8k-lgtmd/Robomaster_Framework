//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"

#include <cmath>

namespace
{
/*
 * PWM 频率的计算链路：
 *   定时器输入时钟 -> 预分频器 PSC -> 计数器时钟 -> 自动重装值 ARR -> PWM 周期。
 * GetTimerClock 返回“定时器输入时钟”，尚未除以 PSC+1。
 *
 * 按本工程 TIM1 配置举例：系统时钟树先决定 TIM1CLK，PSC=167 时计数器
 * 每 168 个 TIM1CLK 产生一个计数步进；ARR=99 时一个周期包含 100 个计数步进。
 * 因而 PWM 频率为 TIM1CLK / 168 / 100。实际频率取决于运行时 RCC 配置。
 *
 * 以上计算假设定时器使用内部时钟和向上计数。GetTimerClock 依据 STM32F407
 * 的 APB 时钟树规则计算输入时钟；若定时器改用外部时钟或其他时钟源，
 * 该结果不再代表实际计数时钟，应同步调整板级时钟配置和此处算法。
 */
uint32_t GetTimerClock(TIM_HandleTypeDef *htim)
{
    RCC_ClkInitTypeDef clock_config = {};
    uint32_t flash_latency;
    /*
     * HAL_RCC_GetClockConfig 返回 APB 分频配置；flash_latency 在这里仅作为
     * HAL API 的输出参数，并不参与定时器时钟计算。
     */
    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

    uint32_t pclk;
    uint32_t divider;
    // STM32F4 的 TIM1/8/9/10/11 挂在 APB2；它们使用 PCLK2 作为总线时钟来源。
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 ||
        htim->Instance == TIM10 || htim->Instance == TIM11)
    {
        pclk = HAL_RCC_GetPCLK2Freq();
        divider = clock_config.APB2CLKDivider;
    }
    // TIM2/3/4/5/6/7/12/13/14 挂在 APB1，使用 PCLK1。
    else if (htim->Instance == TIM2 || htim->Instance == TIM3 || htim->Instance == TIM4 ||
             htim->Instance == TIM5 || htim->Instance == TIM6 || htim->Instance == TIM7 ||
             htim->Instance == TIM12 || htim->Instance == TIM13 || htim->Instance == TIM14)
    {
        pclk = HAL_RCC_GetPCLK1Freq();
        divider = clock_config.APB1CLKDivider;
    }
    else
    {
        // 未识别的定时器无法可靠推导时钟，交由项目统一错误处理，不返回猜测值。
        Error_Handler();
        return 0;
    }

    /*
     * STM32F407 的定时器时钟规则：
     * APB 预分频为 DIV1 时，TIMxCLK=PCLKx；
     * APB 预分频大于 DIV1 时，TIMxCLK=2*PCLKx。
     * 注意它不是无条件等于 HCLK，也不是简单等于对应 PCLK。
     * clock_config 中保存的是 HAL 编码后的分频常量，因此只对 DIV1 做等值判断。
     */
    return divider == RCC_HCLK_DIV1 ? pclk : 2U * pclk;
}

uint32_t GetTimerMax(TIM_HandleTypeDef *htim)
{
    /*
     * ARR 是计数器的自动重装寄存器，其可表示的最大值取决于计数器位宽。
     * STM32F407 的 TIM2/TIM5 为 32 位，其余本工程可能使用的定时器为 16 位。
     * 此处返回 ARR/CCR 可表示的最大计数值，不是“计数次数”；计数次数最大为此值+1。
     */
    return htim->Instance == TIM2 || htim->Instance == TIM5 ? UINT32_MAX : UINT16_MAX;
}
}

/*
 * PWM 外设初始化入口。
 * 当前板级工程只在 CubeMX 中配置并生成 TIM1，因此此函数只初始化 TIM1；
 * 初始化会配置定时器和通道参数，但不等同于启动 PWM 输出通道。
 */
void PWM_Init(void)
{
    MX_TIM1_Init();
}

/*
 * 开始向指定通道输出 PWM 波形。
 * htim 必须指向已由 HAL 初始化的定时器句柄，Channel 使用 HAL 的 TIM_CHANNEL_x 常量。
 */
void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    /*
     * HAL_TIM_PWM_Start 会使能通道输出并启动相应定时器计数。
     * 它不会替调用方配置 GPIO 复用、定时器 PWM 模式或安全占空比，
     * 这些须在 MX_TIMx_Init/板级配置中先准备好。
     * 句柄无效或 HAL 启动失败都进入项目统一错误处理。
     */
    if (htim == nullptr || htim->Instance == nullptr || HAL_TIM_PWM_Start(htim, Channel) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * 停止指定通道的 PWM 输出。该操作只停止输出通道，不销毁/反初始化整个定时器；
 * 若同一计数器还有其他活动通道，应按 HAL 对多通道计时器的语义管理其运行状态。
 */
void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // 与 Start 使用相同的句柄有效性及 HAL 返回值检查。
    if (htim == nullptr || htim->Instance == nullptr || HAL_TIM_PWM_Stop(htim, Channel) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * 把以秒为单位的期望周期换算为硬件 ARR。
 *
 * 向上计数模式的计数次数为 ARR+1；计数频率为 TIMxCLK/(PSC+1)，因此：
 *   period = (PSC+1)*(ARR+1)/TIMxCLK
 *   ARR+1 = period*TIMxCLK/(PSC+1)
 *
 * 本函数只改写自动重装值，不启动 PWM，也不改写预分频器、通道或 GPIO 配置。
 * 若定时器启用了 ARR 预装载，新 ARR 会在定时器更新事件时生效；未启用预装载时，
 * 写入时机可能影响当前周期，因此运行中动态改周期应结合定时器同步策略使用。
 * PSC 保持初始化配置不变；可表示的周期范围由 PSC 和 ARR 位宽共同决定，
 * 如果目标超出 ARR 范围，本函数报错而不自动更改 PSC 或把目标钳位到边界。
 */
void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    // 拒绝空句柄、未绑定外设、NaN/无穷或非正周期，避免无定义的浮点到整数转换。
    if (htim == nullptr || htim->Instance == nullptr || !std::isfinite(period_s) || period_s <= 0.0f)
    {
        Error_Handler();
        return;
    }

    /*
     * ticks 表示期望周期内所需的定时器计数次数，即 ARR+1。
     * 使用 double 做中间运算，减少 float 乘法在长周期/高时钟下的精度损失。
     * PSC 寄存器保存的是“分频系数减一”，所以计数时钟除以 PSC+1。
     */
    const double ticks = static_cast<double>(period_s) * GetTimerClock(htim) /
                         (static_cast<double>(htim->Init.Prescaler) + 1.0);
    const uint32_t max_period = GetTimerMax(htim);
    /*
     * 下界确保至少有一个定时器计数步进；上界将 ARR 限制在硬件寄存器范围内。
     * 注意检查的是理想计数次数，取整在检查之后完成；很接近边界的浮点值最终
     * 仍由 round 得到整数计数次数，再减一作为 ARR。
     * 不可表示的时间不会静默裁剪，因为裁剪后输出周期将与调用者请求不一致。
     */
    if (!std::isfinite(ticks) || ticks < 1.0 || ticks > static_cast<double>(max_period) + 1.0)
    {
        Error_Handler();
        return;
    }

    // ticks 取最近整数计数以减小量化误差，再由计数次数转换回 ARR（计数次数减一）。
    __HAL_TIM_SET_AUTORELOAD(htim, static_cast<uint32_t>(std::round(ticks)) - 1U);
}

/*
 * 以 0.0~1.0 的比例设置某一路输出比较寄存器 CCR。
 * PWM 模式下 CCR 决定有效电平持续的计数数目，ARR+1 是一个完整周期的计数数目。
 * 例如向上计数、PWM mode 1、高电平有效时，计数器小于 CCR 的区间为有效脉冲；
 * 反相输出、向下/中心对齐计数模式会影响波形解释，比例接口不替代极性和模式配置。
 * 调用本函数前应先设置期望周期；若 ARR 尚未配置，结果会基于当前硬件 ARR 计算。
 */
void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    // 拒绝无效句柄、非有限占空比、越界比例和非 CH1~CH4 通道。
    if (htim == nullptr || htim->Instance == nullptr || !std::isfinite(dutyratio) ||
        dutyratio < 0.0f || dutyratio > 1.0f ||
        (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2 &&
         channel != TIM_CHANNEL_3 && channel != TIM_CHANNEL_4))
    {
        Error_Handler();
        return;
    }

    /*
     * 理想比较值 CCR = 占空比*(ARR+1)，四舍五入到最接近的硬件计数。
     * 比例为 0 时得到 CCR=0；比例为 1 时理想值为 ARR+1，而 CCR 本身不能超过
     * 定时器寄存器位宽，因此按计数器最大值裁剪。输出波形的精确端点行为还受
     * 定时器 PWM 模式及芯片比较逻辑定义影响。
     * 可实现的占空比分辨率约为 1/(ARR+1)：周期计数越少，每个计数步进造成的比例变化越大。
     */
    const uint32_t max_compare = GetTimerMax(htim);
    const double pulse = std::round(static_cast<double>(dutyratio) *
                                    (static_cast<double>(__HAL_TIM_GET_AUTORELOAD(htim)) + 1.0));
    /*
     * 32 位计数器的 CCR 可容纳 32 位计数值；16 位定时器需裁剪到 UINT16_MAX。
     * 在 TIMx 计数器已经运行时，CCR 是否立即生效还取决于通道的比较预装载设置。
     */
    __HAL_TIM_SET_COMPARE(htim, channel,
                          static_cast<uint32_t>(pulse > max_compare ? max_compare : pulse));
}
