#include <chrono>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include <colli2de/Registry.hpp>
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Registry | Bullet raycast", "[Registry][Benchmark][Raycast][Bullet]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    auto circles = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    auto translations = generateRandomTranslations(100'000, -5.0f, 5.0f, seed + 1);
    Ray ray{{0.0f, -60.0f}, {0.0f, 60.0f}};

    Registry<uint32_t> registry;
    for(uint32_t i = 0; i < 10'000; ++i)
    {
        registry.createEntity(i, BodyType::Bullet, Transform(circles[i].center));
        registry.addShape(i, Circle{{0.0f,0.0f}, circles[i].radius});
        registry.moveEntity(i, Transform(translations[i]));
    }

    BENCHMARK_FUNCTION("Registry | Bullet raycast among 10k entities", 5ms, [&]()
    {
        return registry.rayCast(ray).size();
    }, [&]()
    {
        registry.clear();
        for(uint32_t i = 0; i < 10'000; ++i)
        {
            registry.createEntity(i, BodyType::Bullet, Transform(circles[i].center));
            registry.addShape(i, Circle{{0.0f,0.0f}, circles[i].radius});
            registry.moveEntity(i, Transform(translations[i]));
        }
    });

    registry.clear();
    for(uint32_t i = 0; i < 100'000; ++i)
    {
        registry.createEntity(i, BodyType::Bullet, Transform(circles[i].center));
        registry.addShape(i, Circle{{0.0f,0.0f}, circles[i].radius});
        registry.moveEntity(i, Transform(translations[i]));
    }

    BENCHMARK_FUNCTION("Registry | Bullet raycast among 100k entities", 30ms, [&]()
    {
        return registry.rayCast(ray).size();
    });
}
