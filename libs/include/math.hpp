#ifndef MATH_HPP
#define MATH_HPP

namespace Numeric
{
    // Clamp input symmetrically to [-abs(maxValue), abs(maxValue)].
    float LimitABS(float input, float maxValue);
}

#endif // MATH_HPP
