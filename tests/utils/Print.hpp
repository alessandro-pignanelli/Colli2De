#include <chrono>

using namespace std::literals::chrono_literals;

#if __has_include(<print>)
    #include <print>
    #define print(...) ::std::print(__VA_ARGS__)
    #define println(...) ::std::println(__VA_ARGS__)
#else
    #include <iostream>
    #include <format>

    #define print(...) (std::cout << std::format(__VA_ARGS__))
    #define println(...) (std::cout << std::format(__VA_ARGS__) << std::endl)
#endif

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
        println("\n---- ERROR ----");
        println("Elapsed time: {}{} (exceeded threshold of {}{})", elapsedUnit, unit, thresholdUnit, unit);
    }
}
