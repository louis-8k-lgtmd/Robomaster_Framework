//
// Created by cosmosmount on 2025/9/2.
//

#ifndef RM26_H7_BSP_PWM_HPP
#define RM26_H7_BSP_PWM_HPP

#include "tim.h"

/**
 * @brief  按板级定时器配置初始化 PWM 外设。
 */
void PWM_Init(void);

/**
 * @brief  启动指定定时器通道的 PWM 输出。
 */
void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);

/**
 * @brief  停止指定定时器通道的 PWM 输出。
 */
void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

/**
 * @brief  按秒设置 PWM 周期，并更新定时器自动重装值。
 * @param  htim 定时器句柄。
 * @param  period 周期，单位秒。
 */
void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period);

/**
 * @brief  设置指定通道的 PWM 占空比。
 * @param  htim 定时器句柄。
 * @param  dutyratio 占空比，范围 0~1。
 * @param  channel HAL 定时器通道常量。
 */
void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel);

#endif //RM26_H7_BSP_PWM_HPP