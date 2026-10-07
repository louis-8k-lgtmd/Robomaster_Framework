#include "bsp.hpp"

#include "bsp_can.hpp"
#include "bsp_usart.hpp"

// BSP 汇总初始化入口；外设时钟和 GPIO 由 main.c/CubeMX 更早配置。
void bsp_Init() {
    USART_Init();
    CAN_Init();
}
