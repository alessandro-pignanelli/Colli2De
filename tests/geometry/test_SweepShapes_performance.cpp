#include <chrono>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include <colli2de/internal/collision/Collision.hpp>
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Sweep performance: Circle vs Circle", "[Sweep][Benchmark][Circle]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto moving = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto targets = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed + 1);
    auto translations = generateRandomTranslations(100'000, -5.0f, 5.0f, seed + 2);

    BENCHMARK_FUNCTION("Sweep 10k circle pairs", 5ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (sweep(moving[i], Transform{},  Transform{translations[i], 0}, targets[i], Transform{}))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Sweep 100k circle pairs", 50ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (sweep(moving[i], Transform{},  Transform{translations[i], 0}, targets[i], Transform{}))
                ++total;
        }
        return total;
    });
}

TEST_CASE("Sweep performance: Polygon vs Polygon", "[Sweep][Benchmark][Polygon]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto moving = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed);
    auto targets = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed + 1);
    auto translations = generateRandomTranslations(100'000, -5.0f, 5.0f, seed + 2);

    BENCHMARK_FUNCTION("Sweep 10k polygon pairs", 10ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (sweep(moving[i], Transform{},  Transform{translations[i], 0}, targets[i], Transform{}))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Sweep 100k polygon pairs", 80ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (sweep(moving[i], Transform{},  Transform{translations[i], 0}, targets[i], Transform{}))
                ++total;
        }
        return total;
    });
}
