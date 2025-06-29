#include <chrono>
#include <cstdint>
#include <iostream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "collision/Collision.hpp"
#include "colli2de/Shapes.hpp"
#include "geometry/Transformations.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Collision performance: Circle vs Circle", "[collision][Benchmark][Circle]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto circlesA = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto circlesB = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK("Collide 10k circle pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            total += collide(circlesA[i], t, circlesB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 10ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/10ms" << std::endl;

    BENCHMARK("Collide 100k circle pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            total += collide(circlesA[i], t, circlesB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 100ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/100ms" << std::endl;
}

TEST_CASE("Collision performance: Segment vs Segment", "[collision][Benchmark][Segment]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto segA = generateRandomSegments(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto segB = generateRandomSegments(100'000, -50.0f, 50.0f, 1.0f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK("Collide 10k segment pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            total += collide(segA[i], t, segB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 10ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/10ms" << std::endl;

    BENCHMARK("Collide 100k segment pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            total += collide(segA[i], t, segB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 100ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/100ms" << std::endl;
}

TEST_CASE("Collision performance: Capsule vs Capsule", "[collision][Benchmark][Capsule]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto capA = generateRandomCapsules(100'000, -50.0f, 50.0f, 1.0f, 0.5f, seed);
    auto capB = generateRandomCapsules(100'000, -50.0f, 50.0f, 1.0f, 0.5f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK("Collide 10k capsule pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            total += collide(capA[i], t, capB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 10ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/10ms" << std::endl;

    BENCHMARK("Collide 100k capsule pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            total += collide(capA[i], t, capB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 100ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/100ms" << std::endl;
}

TEST_CASE("Collision performance: Polygon vs Polygon", "[collision][Benchmark][Polygon]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto polyA = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed);
    auto polyB = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK("Collide 10k polygon pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            total += collide(polyA[i], t, polyB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 10ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/10ms" << std::endl;

    BENCHMARK("Collide 100k polygon pairs")
    {
        const auto start = high_resolution_clock::now();
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            total += collide(polyA[i], t, polyB[i], t).pointCount;
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return total;
    };
    CHECK(elapsed < 100ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/100ms" << std::endl;
}
