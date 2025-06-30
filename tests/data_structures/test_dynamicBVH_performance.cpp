#include <chrono>
#include <cstdint>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "data_structures/DynamicBVH.hpp"
#include "geometry/AABB.hpp"
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("DynamicBVH performance: Bulk insertion", "[DynamicBVH][Benchmark][Insert]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);

    BENCHMARK_FUNCTION("Insert 10k proxies", 10ms, [&]()
    {
        DynamicBVH<uint32_t> bvh;
        for (uint32_t i = 0; i < 10'000; ++i)
            bvh.createProxy(aabbs[i], i);
        return bvh.size();
    });

    BENCHMARK_FUNCTION("Insert 100k proxies", 100ms, [&]()
    {
        DynamicBVH<uint32_t> bvh;
        for (uint32_t i = 0; i < 100'000; ++i)
            bvh.createProxy(aabbs[i], i);
        return bvh.size();
    });
}

TEST_CASE("DynamicBVH performance: Moving proxies", "[DynamicBVH][Benchmark][Move]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);

    DynamicBVH<uint32_t> bvh;
    std::vector<NodeIndex> indices;
    for (uint32_t i = 0; i < 10'000; ++i)
        indices.push_back(bvh.createProxy(aabbs[i], i));

    BENCHMARK_FUNCTION("Move 10k proxies to new location", 300us, [&]()
    {
        for (size_t i = 0; i < indices.size(); ++i)
            bvh.moveProxy(indices[i], aabbs[i].move(Vec2{ 50.0f, 0 }), Vec2{50.0f, 0});
        return bvh.size();
    });

    bvh.clear();
    indices.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        indices.push_back(bvh.createProxy(aabbs[i], i));

    BENCHMARK_FUNCTION("Move 100k proxies to new location", 3ms, [&]()
    {
        for (size_t i = 0; i < indices.size(); ++i)
            bvh.moveProxy(indices[i], aabbs[i].move(Vec2{ 50.0f, 0 }), Vec2{50.0f, 0});
        return bvh.size();
    });
}

TEST_CASE("DynamicBVH performance: Broad-phase AABB query", "[DynamicBVH][Benchmark][Query]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);
    AABB query{Vec2{20, 0}, Vec2{50, 4}};

    DynamicBVH<uint32_t> bvh;
    for (uint32_t i = 0; i < 10'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("Query 10k proxies", 10us, [&]()
    {
        size_t count = 0;
        count += bvh.query(query).size();
        return count;
    });

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("Query 100k proxies", 100us, [&]()
    {
        size_t count = 0;
        count += bvh.query(query).size();
        return count;
    });
}

TEST_CASE("DynamicBVH performance: Piercing raycast", "[DynamicBVH][Benchmark][Raycast]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);
    Ray ray{Vec2{0, 0.5f}, Vec2{100, 34.5f}};

    DynamicBVH<uint32_t> bvh;
    for (uint32_t i = 0; i < 10'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("Raycast through 10k proxies", 20us, [&]()
    {
        auto hits = bvh.piercingRaycast(ray);
        return hits.size();
    });

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("Raycast through 100,000 proxies", 200us, [&]()
    {
        auto hits = bvh.piercingRaycast(ray);
        return hits.size();
    });
}

TEST_CASE("DynamicBVH: BroadPhaseCollisions benchmark (10k random proxies)", "[DynamicBVH][BroadPhaseCollisions][Benchmark]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 1000.0f, 3.0f, seed);

    DynamicBVH<uint32_t> bvh;
    for (uint32_t i = 0; i < 1'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("Find all overlapping pairs among 1k proxies", 50us, [&]()
    {
        const auto pairs = bvh.findBroadPhaseCollisions();
        return pairs.size();
    });

    bvh.clear();
    for (uint32_t i = 0; i < 10'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("Find all overlapping pairs among 10k proxies", 5ms, [&]()
    {
        const auto pairs = bvh.findBroadPhaseCollisions();
        return pairs.size();
    });

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("Find all overlapping pairs among 100k proxies", 100ms, [&]()
    {
        const auto pairs = bvh.findBroadPhaseCollisions();
        return pairs.size();
    });
}
