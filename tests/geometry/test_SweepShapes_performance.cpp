#include <chrono>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "geometry/SweepShapes.hpp"
#include "utils/Print.hpp"
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

    BENCHMARK("Sweep 10k circle pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (sweep(moving[i], Transform{}, translations[i], Rotation{}, targets[i], Transform{}))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 5ms);
    printElapsed(elapsed, 5ms);

    BENCHMARK("Sweep 100k circle pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (sweep(moving[i], Transform{}, translations[i], Rotation{}, targets[i], Transform{}))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 50ms);
    printElapsed(elapsed, 50ms);
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

    BENCHMARK("Sweep 10k polygon pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (sweep(moving[i], Transform{}, translations[i], Rotation{}, targets[i], Transform{}))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 10ms);
    printElapsed(elapsed, 10ms);

    BENCHMARK("Sweep 100k polygon pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (sweep(moving[i], Transform{}, translations[i], Rotation{}, targets[i], Transform{}))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 70ms);
    printElapsed(elapsed, 70ms);
}
