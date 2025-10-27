#include <chrono>

using namespace std::literals::chrono_literals;

#if __has_include(<print>)
#include <print>
#define _print(...) (::std::print(__VA_ARGS__))
#define _println(...) (::std::println(__VA_ARGS__))
#define _printFile(file, ...) (print(file, __VA_ARGS__))
#define _printlnFile(file, ...) (println(file, __VA_ARGS__))
#else
#include <format>
#include <iostream>

#define _print(...) (std::cout << std::format(__VA_ARGS__))
#define _println(...) (std::cout << std::format(__VA_ARGS__) << std::endl)
#define _printFile(file, ...) (file << std::format(__VA_ARGS__))
#define _printlnFile(file, ...) (file << std::format(__VA_ARGS__) << std::endl)
#endif

namespace c2d
{
namespace test
{

template <class... _Types> inline void println(const std::format_string<_Types...> _Fmt, _Types&&... _Args)
{
    _println(_Fmt, std::forward<_Types>(_Args)...);
}

inline void printElapsed(const std::chrono::microseconds& elapsed, const std::chrono::microseconds& threshold)
{
    const auto useMicroseconds = threshold < 1ms;
    const auto elapsedUnit = elapsed.count() / (useMicroseconds ? 1.0f : 1000.0f);
    const auto thresholdUnit = threshold.count() / (useMicroseconds ? 1.0f : 1000.0f);
    const auto unit = useMicroseconds ? "us" : "ms";

    if (elapsed <= threshold)
    {
        println("\nElapsed time: {}{} (within threshold of {}{})", elapsedUnit, unit, thresholdUnit, unit);
    }
    else
    {
#ifdef NDEBUG
        println("\n---- ERROR ----");
#endif
        println("Elapsed time: {}{} (exceeded threshold of {}{})", elapsedUnit, unit, thresholdUnit, unit);
    }
}

} // namespace test
} // namespace c2d
