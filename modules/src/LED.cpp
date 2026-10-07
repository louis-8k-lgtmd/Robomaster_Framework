#include "LED.hpp"
#include "led.h"
#include "gpio.h"

namespace LED
{
/*
 * 板级 RGB LED 连接关系：PH10=红、PH11=绿、PH12=蓝。
 * GPIO 为低电平有效，因此对外接口采用更直观的逻辑语义：
 * 参数 true 表示“点亮”，内部转换为 RESET；false 表示“熄灭”，输出 SET。
 */
void SetRGB(bool red, bool green, bool blue)
{
    // 三色独立控制；一次调用分别更新三个引脚的输出电平。
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, red ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, green ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, blue ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Off()
{
    // 统一复用 SetRGB 的低有效转换，关闭所有颜色通道。
    SetRGB(false, false, false);
}
}

/*
 * C ABI 适配层：让 C 源文件（如 ThreadX 应用）能调用 C++ LED 命名空间实现。
 * 将任意非零值规范为 true，零值规范为 false；不改变硬件引脚映射。
 */
extern "C" void LED_SetRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    LED::SetRGB(red != 0U, green != 0U, blue != 0U);
}

extern "C" void LED_Off(void)
{
    LED::Off();
}
