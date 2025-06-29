#include <chrono>
#include <cstdint>
#include <iostream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "geometry/RaycastShapes.hpp"
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

    BENCHMARK("Raycast 10k circles")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(circles[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 500us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/500us" << std::endl;

    BENCHMARK("Raycast 100k circles")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(circles[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 5000us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/5000us" << std::endl;
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

    BENCHMARK("Raycast 10k segments")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(segments[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 500us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/500us" << std::endl;

    BENCHMARK("Raycast 100k segments")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(segments[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 5000us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/5000us" << std::endl;
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

    BENCHMARK("Raycast 10k capsules")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(capsules[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 1000us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/1000us" << std::endl;

    BENCHMARK("Raycast 100k capsules")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(capsules[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 10000us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/10000us" << std::endl;
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

    BENCHMARK("Raycast 10k polygons")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (raycast(polygons[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 500us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/500us" << std::endl;

    BENCHMARK("Raycast 100k polygons")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (raycast(polygons[i], Transform{}, rays[i]))
                ++total;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 5000us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/5000us" << std::endl;
}
