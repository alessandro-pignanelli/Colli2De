#pragma once

#include <cmath>
#include <thread>

inline constexpr bool float_equals(float val1, float val2, float epsilon = 1e-6f)
{
    return std::abs(val1 - val2) < epsilon;
}

inline constexpr bool float_equals(double val1, double val2, double epsilon = 1e-6)
{
    return std::abs(val1 - val2) < epsilon;
}

#ifdef __APPLE__

#define C2D_PARALLEL_FOR(idxStart, idxEnd, func)                                                                       \
    {                                                                                                                  \
        for (size_t i = (size_t)(idxStart); i < (size_t)(idxEnd); ++i)                                                 \
            func(i);                                                                                                   \
    }

#else

#include <execution>
#include <ranges>
#define C2D_PARALLEL_FOR(idxStart, idxEnd, func)                                                                       \
    {                                                                                                                  \
        std::ranges::iota_view indexes((size_t)(idxStart), (size_t)(idxEnd));                                          \
        std::for_each(std::execution::par_unseq, indexes.begin(), indexes.end(), func);                                \
    }

#endif

#define C2D_RUN_PARALLEL_AND_WAIT(...)                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        const auto _tasks = std::make_tuple(__VA_ARGS__);                                                              \
        std::vector<std::thread> _threads;                                                                             \
        _threads.reserve(std::tuple_size_v<decltype(_tasks)>);                                                         \
        std::apply([&_threads](auto&&... fns) { (_threads.emplace_back(std::forward<decltype(fns)>(fns)), ...); },     \
                   _tasks);                                                                                            \
        for (auto& _t : _threads)                                                                                      \
            _t.join();                                                                                                 \
    } while (0)
#define C2D_RUN_AND_WAIT(...)                                                                                          \
    do                                                                                                                 \
    {                                                                                                                  \
        const auto _tasks = std::make_tuple(__VA_ARGS__);                                                              \
        std::apply([](auto&&... fns) { (std::forward<decltype(fns)>(fns)(), ...); }, _tasks);                          \
    } while (0)

namespace c2d
{
struct _Empty
{
};
} // namespace c2d

#define CONDITIONAL(condition, type) [[no_unique_address]] std::conditional_t<(condition), type, c2d::_Empty>
