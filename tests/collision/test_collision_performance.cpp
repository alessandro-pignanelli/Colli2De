#include <chrono>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include <colli2de/internal/collision/Collision.hpp>
#include <colli2de/Transform.hpp>
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Collision performance: Circle vs Circle", "[collision][Benchmark][Circle]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto circlesA = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto circlesB = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK_FUNCTION("Collide 10k circle pairs", 300us, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
            total += collide(circlesA[i], t, circlesB[i], t).pointCount;
        return total;
    });

    BENCHMARK_FUNCTION("Collide 100k circle pairs", 3ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
            total += collide(circlesA[i], t, circlesB[i], t).pointCount;
        return total;
    });
}

TEST_CASE("Collision performance: Segment vs Segment", "[collision][Benchmark][Segment]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto segA = generateRandomSegments(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto segB = generateRandomSegments(100'000, -50.0f, 50.0f, 1.0f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK_FUNCTION("Collide 10k segment pairs", 700us, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
            total += collide(segA[i], t, segB[i], t).pointCount;
        return total;
    });

    BENCHMARK_FUNCTION("Collide 100k segment pairs", 10ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
            total += collide(segA[i], t, segB[i], t).pointCount;
        return total;
    });
}

TEST_CASE("Collision performance: Capsule vs Capsule", "[collision][Benchmark][Capsule]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto capA = generateRandomCapsules(100'000, -50.0f, 50.0f, 1.0f, 0.5f, seed);
    auto capB = generateRandomCapsules(100'000, -50.0f, 50.0f, 1.0f, 0.5f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK_FUNCTION("Collide 10k capsule pairs", 700us, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
            total += collide(capA[i], t, capB[i], t).pointCount;
        return total;
    });

    BENCHMARK_FUNCTION("Collide 100k capsule pairs", 10ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
            total += collide(capA[i], t, capB[i], t).pointCount;
        return total;
    });
}

TEST_CASE("Collision performance: Polygon vs Polygon", "[collision][Benchmark][Polygon]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto polyA = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed);
    auto polyB = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed + 1);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    BENCHMARK_FUNCTION("Collide 10k polygon pairs", 500us, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
            total += collide(polyA[i], t, polyB[i], t).pointCount;
        return total;
    });

    BENCHMARK_FUNCTION("Collide 100k polygon pairs", 5ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
            total += collide(polyA[i], t, polyB[i], t).pointCount;
        return total;
    });
}
