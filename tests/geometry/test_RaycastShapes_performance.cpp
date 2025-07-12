#include <chrono>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include <colli2de/internal/geometry/RaycastShapes.hpp>
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Raycast performance: Circle", "[Raycast][Benchmark][Circle]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto circles = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto rays = generateRandomRays(100'000, -60.0f, 60.0f, seed + 1);

    BENCHMARK_FUNCTION("Raycast 10k circles", 500us, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(circles[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Raycast 100k circles", 5ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(circles[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });
}

TEST_CASE("Raycast performance: Segment", "[Raycast][Benchmark][Segment]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto segments = generateRandomSegments(100'000, -50.0f, 50.0f, 4.0f, seed);
    auto rays = generateRandomRays(100'000, -60.0f, 60.0f, seed + 1);

    BENCHMARK_FUNCTION("Raycast 10k segments", 500us, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(segments[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Raycast 100k segments", 5ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(segments[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });
}

TEST_CASE("Raycast performance: Capsule", "[Raycast][Benchmark][Capsule]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto capsules = generateRandomCapsules(100'000, -50.0f, 50.0f, 4.0f, 1.0f, seed);
    auto rays = generateRandomRays(100'000, -60.0f, 60.0f, seed + 1);

    BENCHMARK_FUNCTION("Raycast 10k capsules", 3ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(capsules[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Raycast 100k capsules", 15ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(capsules[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });
}

TEST_CASE("Raycast performance: Polygon", "[Raycast][Benchmark][Polygon]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto polygons = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed);
    auto rays = generateRandomRays(100'000, -60.0f, 60.0f, seed + 1);

    BENCHMARK_FUNCTION("Raycast 10k polygons", 1ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(polygons[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Raycast 100k polygons", 10ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(polygons[i], Transform{}, rays[i]))
                ++total;
        }
        return total;
    });
}
