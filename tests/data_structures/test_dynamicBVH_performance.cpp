#include <algorithm>
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
using RaycastInfo = DynamicBVH<uint32_t>::RaycastInfo;

TEST_CASE("DynamicBVH | Bulk insertion", "[DynamicBVH][Benchmark][Insert]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);

    BENCHMARK_FUNCTION("DynamicBVH | Insert 10k proxies", 10ms, [&]()
    {
        DynamicBVH<uint32_t> bvh;
        for (uint32_t i = 0; i < 10'000; ++i)
            bvh.createProxy(aabbs[i], i);
        return bvh.size();
    });

    BENCHMARK_FUNCTION("DynamicBVH | Insert 100k proxies", 130ms, [&]()
    {
        DynamicBVH<uint32_t> bvh;
        for (uint32_t i = 0; i < 100'000; ++i)
            bvh.createProxy(aabbs[i], i);
        return bvh.size();
    });
}

TEST_CASE("DynamicBVH | Moving proxies", "[DynamicBVH][Benchmark][Move]")
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

    BENCHMARK_FUNCTION("DynamicBVH | Move 10k proxies to new location", 300us, [&]()
    {
        for (size_t i = 0; i < indices.size(); ++i)
            bvh.moveProxy(indices[i], aabbs[i].move(Vec2{ 50.0f, 0 }), Vec2{50.0f, 0});
        return bvh.size();
    });

    bvh.clear();
    indices.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        indices.push_back(bvh.createProxy(aabbs[i], i));

    BENCHMARK_FUNCTION("DynamicBVH | Move 100k proxies to new location", 3ms, [&]()
    {
        for (size_t i = 0; i < indices.size(); ++i)
            bvh.moveProxy(indices[i], aabbs[i].move(Vec2{ 50.0f, 0 }), Vec2{50.0f, 0});
        return bvh.size();
    });
}

TEST_CASE("DynamicBVH | Broad-phase AABB query", "[DynamicBVH][Benchmark][Query]")
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

    BENCHMARK_FUNCTION("DynamicBVH | Query 10k proxies", 50us, [&]()
    {
        std::set<uint32_t> foundIds;
        bvh.query(query, foundIds);
        return foundIds.size();
    });

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("DynamicBVH | Query 100k proxies", 500us, [&]()
    {
        std::set<uint32_t> foundIds;
        bvh.query(query, foundIds);
        return foundIds.size();
    });

    std::vector<AABB> queries;
    for (uint32_t i = 0; i < 10'000; ++i)
    {
        float x = static_cast<float>(i % 100);
        float y = static_cast<float>(i % 50);
        queries.emplace_back(Vec2{x, y}, Vec2{x + 1.0f, y + 1.0f});
    }
    std::shuffle(queries.begin(), queries.end(), std::mt19937{std::random_device{}()});

    BENCHMARK_FUNCTION("DynamicBVH | 10k Queries 100k proxies", 300ms, [&]()
    {
        std::set<uint32_t> foundIds;
        for (const auto& query : queries)
            bvh.query(query, foundIds);
        return foundIds.size();
    });

    BENCHMARK_FUNCTION("DynamicBVH | 10k Batch Queries 100k proxies", 80ms, [&]()
    {
        std::vector<std::set<uint32_t>> results = bvh.batchQuery(queries, 24);
        return results.size();
    });
}

TEST_CASE("DynamicBVH | Piercing raycast", "[DynamicBVH][Benchmark][Raycast]")
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

    BENCHMARK_FUNCTION("DynamicBVH | Raycast through 10k proxies", 200us, [&]()
    {
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits);
        return hits.size();
    });

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("DynamicBVH | Raycast through 100k proxies", 2000us, [&]()
    {
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits);
        return hits.size();
    });
}

TEST_CASE("DynamicBVH | BroadPhaseCollisions benchmark (10k random proxies)", "[DynamicBVH][BroadPhaseCollisions][Benchmark]")
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

    std::set<std::pair<uint32_t, uint32_t>> pairs;

    BENCHMARK_FUNCTION("DynamicBVH | Find all overlapping pairs among 1k proxies", 100us, [&]()
    {
        pairs.clear();
        bvh.findAllCollisions(pairs);
        return pairs.size();
    });

    bvh.clear();
    for (uint32_t i = 0; i < 10'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("DynamicBVH | Find all overlapping pairs among 10k proxies", 5ms, [&]()
    {
        pairs.clear();
        bvh.findAllCollisions(pairs);
        return pairs.size();
    });

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK_FUNCTION("DynamicBVH | Find all overlapping pairs among 100k proxies", 150ms, [&]()
    {
        pairs.clear();
        bvh.findAllCollisions(pairs);
        return pairs.size();
    });
}
