#ifndef RM26_LED_H
#define RM26_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LED_SetRGB(uint8_t red, uint8_t green, uint8_t blue);
void LED_Off(void);

#ifdef __cplusplus
}
#endif

#endif
