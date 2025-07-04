#include <algorithm>
#include <chrono>
#include <cstdint>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;
using BPTRaycastInfo = RaycastInfo<uint32_t>;

TEST_CASE("BroadPhaseTree | Bulk insertion", "[BroadPhaseTree][Benchmark][Insert]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);

    BENCHMARK_FUNCTION("BroadPhaseTree | Insert 10k proxies", 10ms, [&]()
    {
        BroadPhaseTree<uint32_t> tree;
        for (uint32_t i = 0; i < 10'000; ++i)
            tree.addProxy(i, aabbs[i]);
        return tree.size();
    });

    BENCHMARK_FUNCTION("BroadPhaseTree | Insert 100k proxies", 130ms, [&]()
    {
        BroadPhaseTree<uint32_t> tree;
        for (uint32_t i = 0; i < 100'000; ++i)
            tree.addProxy(i, aabbs[i]);
        return tree.size();
    });
}

TEST_CASE("BroadPhaseTree | Moving proxies", "[BroadPhaseTree][Benchmark][Move]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);

    BroadPhaseTree<uint32_t> tree;
    std::vector<BroadPhaseTreeHandle> indices;
    for (uint32_t i = 0; i < 10'000; ++i)
        indices.push_back(tree.addProxy(i, aabbs[i]));

    BENCHMARK_FUNCTION("BroadPhaseTree | Move 10k proxies to new location", 800us, [&]()
    {
        for (size_t i = 0; i < indices.size(); ++i)
            tree.moveProxy(indices[i], aabbs[i].move(Vec2{50.0f, 0}));
        return tree.size();
    });

    BroadPhaseTree<uint32_t> treeLarge;
    indices.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        indices.push_back(treeLarge.addProxy(i, aabbs[i]));

    BENCHMARK_FUNCTION("BroadPhaseTree | Move 100k proxies to new location", 8ms, [&]()
    {
        for (size_t i = 0; i < indices.size(); ++i)
            treeLarge.moveProxy(indices[i], aabbs[i].move(Vec2{50.0f, 0}));
        return treeLarge.size();
    });
}

TEST_CASE("BroadPhaseTree | Broad-phase AABB query", "[BroadPhaseTree][Benchmark][Query]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);
    AABB query{Vec2{20, 0}, Vec2{50, 4}};

    BroadPhaseTree<uint32_t> tree;
    for (uint32_t i = 0; i < 10'000; ++i)
        tree.addProxy(i, aabbs[i]);

    BENCHMARK_FUNCTION("BroadPhaseTree | Query 10k proxies", 50us, [&]()
    {
        std::set<uint32_t> foundIds = tree.query(query);
        return foundIds.size();
    });

    BroadPhaseTree<uint32_t> treeLargeQuery;
    for (uint32_t i = 0; i < 100'000; ++i)
        treeLargeQuery.addProxy(i, aabbs[i]);

    BENCHMARK_FUNCTION("BroadPhaseTree | Query 100k proxies", 500us, [&]()
    {
        std::set<uint32_t> foundIds = treeLargeQuery.query(query);
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

    BENCHMARK_FUNCTION("BroadPhaseTree | 10k Queries 100k proxies", 200ms, [&]()
    {
        std::set<uint32_t> foundIds;
        for (const auto& query : queries)
            foundIds = tree.query(query);
        return foundIds.size();
    });

    BENCHMARK_FUNCTION("BroadPhaseTree | 10k Batch Queries 100k proxies", 80ms, [&]()
    {
        std::vector<std::set<uint32_t>> results = tree.batchQuery(queries, 24);
        return results.size();
    });
}

TEST_CASE("BroadPhaseTree | Piercing raycast", "[BroadPhaseTree][Benchmark][Raycast]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);
    Ray ray{Vec2{0, 0.5f}, Vec2{100, 34.5f}};

    BroadPhaseTree<uint32_t> tree;
    for (uint32_t i = 0; i < 10'000; ++i)
        tree.addProxy(i, aabbs[i]);

    BENCHMARK_FUNCTION("BroadPhaseTree | Raycast through 10k proxies", 200us, [&]()
    {
        std::set<BPTRaycastInfo> hits = tree.piercingRaycast(ray);
        return hits.size();
    });

    BroadPhaseTree<uint32_t> treeLargeRay;
    for (uint32_t i = 0; i < 100'000; ++i)
        treeLargeRay.addProxy(i, aabbs[i]);

    BENCHMARK_FUNCTION("BroadPhaseTree | Raycast through 100,000 proxies", 2000us, [&]()
    {
        std::set<BPTRaycastInfo> hits = treeLargeRay.piercingRaycast(ray);
        return hits.size();
    });
}

TEST_CASE("BroadPhaseTree | BroadPhaseCollisions benchmark (10k random proxies)", "[BroadPhaseTree][BroadPhaseCollisions][Benchmark]")
{
#ifndef NDEBUG
    SKIP("Performance test skipped in Debug mode.");
#endif

    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    microseconds elapsed;
    std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 1000.0f, 3.0f, seed);

    BroadPhaseTree<uint32_t> tree;
    for (uint32_t i = 0; i < 1'000; ++i)
        tree.addProxy(i, aabbs[i]);

    std::set<std::pair<uint32_t, uint32_t>> pairs;

    BENCHMARK_FUNCTION("BroadPhaseTree | Find all overlapping pairs among 1k proxies", 100us, [&]()
    {
        pairs = tree.findAllCollisions();
        return pairs.size();
    });

    BroadPhaseTree<uint32_t> treePairs10k;
    for (uint32_t i = 0; i < 10'000; ++i)
        treePairs10k.addProxy(i, aabbs[i]);

    BENCHMARK_FUNCTION("BroadPhaseTree | Find all overlapping pairs among 10k proxies", 5ms, [&]()
    {
        pairs = treePairs10k.findAllCollisions();
        return pairs.size();
    });

    BroadPhaseTree<uint32_t> treePairs100k;
    for (uint32_t i = 0; i < 100'000; ++i)
        treePairs100k.addProxy(i, aabbs[i]);

    BENCHMARK_FUNCTION("BroadPhaseTree | Find all overlapping pairs among 100k proxies", 150ms, [&]()
    {
        pairs = treePairs100k.findAllCollisions();
        return pairs.size();
    });
}
