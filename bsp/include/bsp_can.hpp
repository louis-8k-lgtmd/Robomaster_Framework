#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 配置并启动 CAN1/CAN2，安装接收过滤器和中断通知。
 * @return 所有 CAN 控制器成功启动时为非零，否则为零。
 */
uint8_t CAN_Init(void);

/**
 * @brief 发送CAN数据。
 * @param hcan 指向CAN句柄的指针，用于配置CAN传输。
 * @param StdId CAN消息的标准标识符
 * @param msg 发送的数据
 * @param len 数据长度，标准数据帧支持 0~8 字节。
 * @note 该函数用于发送CAN数据，目前只支持标准帧
 * @return HAL 邮箱成功接收发送请求时为非零；句柄、控制器状态或参数无效时为零。
 */
uint8_t CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // BSP_CAN_HPP
