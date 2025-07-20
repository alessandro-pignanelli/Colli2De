#include <chrono>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include <colli2de/internal/collision/Collision.hpp>
#include <colli2de/internal/geometry/RaycastShapes.hpp>
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Sweep raycast performance: Circle", "[Sweep][Benchmark][Raycast]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto circles = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto translations = generateRandomTranslations(100'000, -5.0f, 5.0f, seed + 1);
    auto rays = generateRandomRays(100'000, -60.0f, 60.0f, seed + 2);

    BENCHMARK_FUNCTION("Sweep raycast 10k circles", 5ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (sweep(circles[i], Transform{}, Transform{translations[i], 0}, rays[i]))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Sweep raycast 100k circles", 50ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (sweep(circles[i], Transform{}, Transform{translations[i], 0}, rays[i]))
                ++total;
        }
        return total;
    });
}

TEST_CASE("Sweep raycast performance: Polygon", "[SweepRaycast][Benchmark][Polygon]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    auto polygons = generateRandomRectangles(100'000, -50.0f, 50.0f, 0.5f, seed);
    auto rays = generateRandomRays(100'000, -60.0f, 60.0f, seed + 1);
    auto translations = generateRandomTranslations(100'000, -5.0f, 5.0f, seed + 2);

    BENCHMARK_FUNCTION("Sweep raycast 10k polygons", 10ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 10'000; ++i)
        {
            if (sweep(polygons[i], Transform{}, Transform{translations[i], 0}, rays[i]))
                ++total;
        }
        return total;
    });

    BENCHMARK_FUNCTION("Sweep raycast 100k polygons", 50ms, [&]()
    {
        uint32_t total = 0;
        for (uint32_t i = 0; i < 100'000; ++i)
        {
            if (sweep(polygons[i], Transform{}, Transform{translations[i], 0}, rays[i]))
                ++total;
        }
        return total;
    });
}
