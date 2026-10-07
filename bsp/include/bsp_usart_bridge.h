#ifndef BSP_USART_BRIDGE_H
#define BSP_USART_BRIDGE_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start USART1/USART6 Receive-to-Idle DMA after the ThreadX scheduler starts. */
void USART_DMA_Start(void);

HAL_StatusTypeDef USART_DMA_Transmit(UART_HandleTypeDef *huart, uint8_t *data,
                                     uint16_t size);

/* Called from the UART interrupt before its DMA buffer is reused. */
void USART_DMA_RxFrame(UART_HandleTypeDef *huart, const uint8_t *data,
                       uint16_t size);

/* Runs in the USART communication thread; override to handle protocol data. */
void USART_ProcessFrame(UART_HandleTypeDef *huart, const uint8_t *data,
                        uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
