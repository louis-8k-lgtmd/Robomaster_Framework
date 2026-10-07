#ifndef RM26_LED_HPP
#define RM26_LED_HPP

namespace LED
{
// true turns on the corresponding channel; hardware pins are active-low.
void SetRGB(bool red, bool green, bool blue);
void Off();
}

#endif // RM26_LED_HPP
