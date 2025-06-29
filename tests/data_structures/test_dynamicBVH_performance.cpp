#include <chrono>
#include <cstdint>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "data_structures/DynamicBVH.hpp"
#include "geometry/AABB.hpp"
#include "utils/Print.hpp"
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

    BENCHMARK("Insert 10,000 proxies")
    {
        const auto start = high_resolution_clock::now();
        DynamicBVH<uint32_t> bvh;
        for (uint32_t i = 0; i < 10'000; ++i)
            bvh.createProxy(aabbs[i], i);
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
    };
    CHECK(elapsed < 10ms);
    printElapsed(elapsed, 10ms);

    BENCHMARK("Insert 100,000 proxies")
    {
        const auto start = high_resolution_clock::now();
        DynamicBVH<uint32_t> bvh;
        for (uint32_t i = 0; i < 100'000; ++i)
            bvh.createProxy(aabbs[i], i);
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
    };
    CHECK(elapsed < 100ms);
    printElapsed(elapsed, 100ms);
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

    BENCHMARK("Move 10,000 proxies to new location")
    {
        const auto start = high_resolution_clock::now();
        for (size_t i = 0; i < indices.size(); ++i)
        {
            bvh.moveProxy(indices[i], aabbs[i].move(Vec2{ 50.0f, 0 }), Vec2{50.0f, 0});
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
    };
    CHECK(elapsed < 80us);
    printElapsed(elapsed, 80us);

    bvh.clear();
    indices.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        indices.push_back(bvh.createProxy(aabbs[i], i));

    BENCHMARK("Move 100,000 proxies to new location")
    {
        const auto start = high_resolution_clock::now();
        for (size_t i = 0; i < indices.size(); ++i)
        {
            bvh.moveProxy(indices[i], aabbs[i].move(Vec2{ 50.0f, 0 }), Vec2{50.0f, 0});
        }
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
    };
    CHECK(elapsed < 800us);
    printElapsed(elapsed, 800us);
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

    BENCHMARK("Query 10,000 proxies")
    {
        const auto start = high_resolution_clock::now();
        size_t count = 0;
        count += bvh.query(query).size();
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return count;
    };
    CHECK(elapsed < 10us);
    printElapsed(elapsed, 10us);

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK("Query 100,000 proxies")
    {
        const auto start = high_resolution_clock::now();
        size_t count = 0;
        count += bvh.query(query).size();
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return count;
    };
    CHECK(elapsed < 100us);
    printElapsed(elapsed, 100us);
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

    BENCHMARK("Raycast through 10,000 proxies")
    {
        const auto start = high_resolution_clock::now();
        auto hits = bvh.piercingRaycast(ray);
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return hits.size();
    };
    CHECK(elapsed < 20us);
    printElapsed(elapsed, 20us);

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK("Raycast through 100,000 proxies")
    {
        const auto start = high_resolution_clock::now();
        auto hits = bvh.piercingRaycast(ray);
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return hits.size();
    };
    CHECK(elapsed < 200us);
    printElapsed(elapsed, 200us);
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

    BENCHMARK("Find all overlapping pairs among 1k proxies")
    {
        const auto start = high_resolution_clock::now();
        const auto pairs = bvh.findBroadPhaseCollisions();
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return pairs.size(); // Return to prevent optimization
    };

    CHECK(elapsed < 50us);
    printElapsed(elapsed, 50us);

    bvh.clear();
    for (uint32_t i = 0; i < 10'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK("Find all overlapping pairs among 10k proxies")
    {
        const auto start = high_resolution_clock::now();
        const auto pairs = bvh.findBroadPhaseCollisions();
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return pairs.size(); // Return to prevent optimization
    };

    CHECK(elapsed < 5ms);
    printElapsed(elapsed, 5ms);

    bvh.clear();
    for (uint32_t i = 0; i < 100'000; ++i)
        bvh.createProxy(aabbs[i], i);

    BENCHMARK("Find all overlapping pairs among 100k proxies")
    {
        const auto start = high_resolution_clock::now();
        const auto pairs = bvh.findBroadPhaseCollisions();
        const auto end = high_resolution_clock::now();
        elapsed = duration_cast<microseconds>(end - start);
        return pairs.size(); // Return to prevent optimization
    };

    CHECK(elapsed < 100ms);
    printElapsed(elapsed, 100ms);
}
