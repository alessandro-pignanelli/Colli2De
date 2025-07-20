#pragma once

#include <cmath>

inline constexpr bool float_equals(float val1, float val2, float epsilon = 1e-6f)
{
    return std::abs(val1 - val2) < epsilon;
}
inline constexpr bool float_equals(double val1, double val2, double epsilon = 1e-6)
{
    return std::abs(val1 - val2) < epsilon;
}

#ifdef __APPLE__

#include <oneapi/tbb/parallel_for.h>
#define C2D_PARALLEL_FOR(idxStart, idxEnd, func) oneapi::tbb::parallel_for((size_t)idxStart, (size_t)idxEnd, func);

#else

#include <execution>
#include <ranges>
#define C2D_PARALLEL_FOR(idxStart, idxEnd, func) { \
    std::ranges::iota_view indexes((size_t)(idxStart), (size_t)(idxEnd)); \
    std::for_each(std::execution::par_unseq, indexes.begin(), indexes.end(), func); \
}

#endif
