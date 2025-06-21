#pragma once

#include <cmath>

inline constexpr bool float_equals(float val1, float val2, float epsilon = 0.0001f)
{
    return std::abs(val1 - val2) < epsilon;
}
inline constexpr bool float_equals(double val1, double val2, double epsilon = 0.000001)
{
    return std::abs(val1 - val2) < epsilon;
}
