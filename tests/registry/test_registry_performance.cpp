#include <colli2de/internal/data_structures/BroadPhaseTree.hpp>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include <colli2de/Registry.hpp>
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("Registry | Bulk insertion", "[Registry][Benchmark][Insert]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    std::vector<Circle> circles = generateRandomCircles(100'000, -50.0f, 50.0f, 1.0f, seed);
    std::vector<Transform> transforms;
    transforms.reserve(100'000);
    for(size_t i = 0; i < circles.size(); ++i)
        transforms.push_back(Transform(circles[i].center));
    
    BENCHMARK_FUNCTION("Registry | Insert 10k entities and shapes", 15ms, ([&]()
    {
        Registry<uint32_t, PartitioningMethod::Grid> reg(64);

        for(uint32_t i = 0; i < 10'000; ++i)
        {
            reg.createEntity(i, BodyType::Dynamic, transforms[i]);
            reg.addShape(i, circles[i]);
        }
        return reg.size();
    }));

    BENCHMARK_FUNCTION("Registry | Insert 100k entities and shapes", 150ms, ([&]()
    {
        Registry<uint32_t, PartitioningMethod::Grid> reg(64);

        for(uint32_t i = 0; i < 100'000; ++i)
        {
            reg.createEntity(i, BodyType::Dynamic, transforms[i]);
            reg.addShape(i, circles[i]);
        }
        return reg.size();
    }));
}

TEST_CASE("Registry | Move entities", "[Registry][Benchmark][Move]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    std::vector<Circle> circles = generateRandomCircles(100'000, -200.0f, 200.0f, 1.0f, seed);
    std::vector<Vec2> translations = generateRandomTranslations(100'000, -4.0f, 4.0f, seed + 1);

    Registry<uint32_t, PartitioningMethod::Grid> reg;
    for(uint32_t i = 0; i < 10'000; ++i)
    {
        reg.createEntity(i, BodyType::Dynamic, Transform(circles[i].center));
        reg.addShape(i, circles[i]);
    }

    BENCHMARK_FUNCTION("Registry | Move 10k entities", 5ms, [&]()
    {
        for(uint32_t i = 0; i < 10'000; ++i)
            reg.moveEntity(i, Transform(translations[i]));
        return reg.size();
    }, [&]()
    {
        reg.clear();
        for (size_t i = 0; i < 10'000; ++i)
        {
            reg.createEntity(i, BodyType::Dynamic, Transform(circles[i].center));
            reg.addShape(i, circles[i]);
        }
    });

    reg.clear();
    for(uint32_t i = 0; i < 100'000; ++i)
    {
        reg.createEntity(i, BodyType::Dynamic, Transform(circles[i].center));
        reg.addShape(i, circles[i]);
    }

    BENCHMARK_FUNCTION("Registry | Move 100k entities", 80ms, [&]()
    {
        for(uint32_t i = 0; i < 100'000; ++i)
            reg.moveEntity(i, Transform(translations[i]));
        return reg.size();
    }, [&]()
    {
        reg.clear();
        for (size_t i = 0; i < 100'000; ++i)
        {
            reg.createEntity(i, BodyType::Dynamic, Transform(circles[i].center));
            reg.addShape(i, circles[i]);
        }
    });
}

TEST_CASE("Registry | Collision query dynamic", "[Registry][Query][AllPairs][Benchmark]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    std::vector<Circle> circles = generateRandomCircles(100'000, -1920.0f, 3840.0f, 12.0f, seed);
    Registry<uint32_t, PartitioningMethod::Grid> reg;
    for(uint32_t i = 0; i < 1'000; ++i)
    {
        const bool isBullet = (i % 1000) == 0;
        const bool isStatic = !isBullet && (i % 3) == 0;
        const BodyType type = (isStatic) ? BodyType::Static :
                        (isBullet) ? BodyType::Bullet : BodyType::Dynamic;
        reg.createEntity(i, type, Transform(circles[i].center));
        reg.addShape(i, Circle({0, 0}, circles[i].radius));
        reg.moveEntity(i, Translation(3.0f, 3.0f));
    }

    BENCHMARK_FUNCTION("Registry | Find all colliding pairs among 1k entities", 30us, [&]()
    {
        return reg.getCollidingPairs().size();
    });

    reg.clear();
    for(uint32_t i = 0; i < 10'000; ++i)
    {
        const bool isBullet = (i % 1000) == 0;
        const bool isStatic = !isBullet && (i % 3) == 0;
        const BodyType type = (isStatic) ? BodyType::Static :
                        (isBullet) ? BodyType::Bullet : BodyType::Dynamic;
        reg.createEntity(i, type, Transform(circles[i].center));
        reg.addShape(i, Circle({0, 0}, circles[i].radius));
        reg.moveEntity(i, Translation(3.0f, 3.0f));
    }

    BENCHMARK_FUNCTION("Registry | Find all colliding pairs among 10k entities", 3ms, [&]()
    {
        return reg.getCollidingPairs().size();
    });

    reg.clear();
    for(uint32_t i = 0; i < 100'000; ++i)
    {
        const bool isBullet = (i % 1000) == 0;
        const bool isStatic = !isBullet && (i % 3) == 0;
        const BodyType type = (isStatic) ? BodyType::Static :
                        (isBullet) ? BodyType::Bullet : BodyType::Dynamic;
        reg.createEntity(i, type, Transform(circles[i].center));
        reg.addShape(i, Circle({0, 0}, circles[i].radius));
        reg.moveEntity(i, Translation(3.0f, 3.0f));
    }

    BENCHMARK_FUNCTION("Registry | Find all colliding pairs among 100k entities", 280ms, [&]()
    {
        return reg.getCollidingPairs().size();
    });
}

TEST_CASE("Registry | Collision query bullets", "[Registry][Query][AllPairs][Benchmark]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    std::vector<Circle> circles = generateRandomCircles(100'000, -1920.0f, 3840.0f, 12.0f, seed);
    Registry<uint32_t, PartitioningMethod::Grid> reg;
    for(uint32_t i = 0; i < 1'000; ++i)
    {
        const BodyType type = BodyType::Bullet;
        reg.createEntity(i, type, Transform(circles[i].center));
        reg.addShape(i, Circle({0, 0}, circles[i].radius));
        reg.moveEntity(i, Translation(3.0f, 3.0f));
    }

    BENCHMARK_FUNCTION("Registry | Find all colliding pairs among 1k bullets", 30us, [&]()
    {
        return reg.getCollidingPairs().size();
    });

    reg.clear();
    for(uint32_t i = 0; i < 10'000; ++i)
    {
        const BodyType type = BodyType::Bullet;
        reg.createEntity(i, type, Transform(circles[i].center));
        reg.addShape(i, Circle({0, 0}, circles[i].radius));
        reg.moveEntity(i, Translation(3.0f, 3.0f));
    }

    BENCHMARK_FUNCTION("Registry | Find all colliding pairs among 10k bullets", 5ms, [&]()
    {
        return reg.getCollidingPairs().size();
    });

    reg.clear();
    for(uint32_t i = 0; i < 100'000; ++i)
    {
        const BodyType type = BodyType::Bullet;
        reg.createEntity(i, type, Transform(circles[i].center));
        reg.addShape(i, Circle({0, 0}, circles[i].radius));
        reg.moveEntity(i, Translation(3.0f, 3.0f));
    }

    BENCHMARK_FUNCTION("Registry | Find all colliding pairs among 100k bullets", 600ms, [&]()
    {
        return reg.getCollidingPairs().size();
    });
}
