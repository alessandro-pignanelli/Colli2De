#include <chrono>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "colli2de/Registry.hpp"
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Registry | Bulk insertion", "[Registry][Benchmark][Insert]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    std::vector<Circle> circles = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    std::vector<Transform> transforms;
    transforms.reserve(100'000);
    for(size_t i = 0; i < circles.size(); ++i)
        transforms.push_back(Transform(circles[i].center));

    BENCHMARK_FUNCTION("Registry | Insert 10k entities", 40ms, [&]()
    {
        Registry<uint32_t> reg;
        for(uint32_t i = 0; i < 10'000; ++i)
        {
            reg.createEntity(i, BodyType::Dynamic, transforms[i]);
            reg.addShape(i, circles[i]);
        }
        return reg.getCollidingPairs().size();
    });

    BENCHMARK_FUNCTION("Registry | Insert 100k entities", 300ms, [&]()
    {
        Registry<uint32_t> reg;
        for(uint32_t i = 0; i < 100'000; ++i)
        {
            reg.createEntity(i, BodyType::Dynamic, transforms[i]);
            reg.addShape(i, circles[i]);
        }
        return reg.getCollidingPairs().size();
    });
}

TEST_CASE("Registry | Move entities", "[Registry][Benchmark][Move]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    std::vector<Circle> circles = generateRandomCircles(10'000, -50.0f, 50.0f, 1.0f, seed);
    std::vector<Vec2> translations = generateRandomTranslations(10'000, -1.0f, 1.0f, seed + 1);

    Registry<uint32_t> reg;
    for(uint32_t i = 0; i < 10'000; ++i)
    {
        reg.createEntity(i, BodyType::Dynamic, Transform(circles[i].center));
        reg.addShape(i, circles[i]);
    }

    BENCHMARK_FUNCTION("Registry | Move 10k entities", 2ms, [&]()
    {
        for(uint32_t i = 0; i < 10'000; ++i)
            reg.moveEntity(i, Transform(translations[i]));
        return reg.getCollidingPairs().size();
    });
}

TEST_CASE("Registry | Collision query", "[Registry][Benchmark][Query]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    std::vector<Circle> circles = generateRandomCircles(10'000, -50.0f, 50.0f, 1.0f, seed);
    Registry<uint32_t> reg;
    for(uint32_t i = 0; i < 10'000; ++i)
    {
        reg.createEntity(i, BodyType::Dynamic, Transform(circles[i].center));
        reg.addShape(i, circles[i]);
    }

    BENCHMARK_FUNCTION("Registry | Find all colliding pairs among 10k entities", 20ms, [&]()
    {
        return reg.getCollidingPairs().size();
    });
}

