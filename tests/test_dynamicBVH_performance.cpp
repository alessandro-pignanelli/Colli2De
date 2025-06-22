#include <chrono>
#include <cstdint>
#include <iostream>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "bvh/dynamicBVH.hpp"
#include "geometry/AABB.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;
using namespace std::chrono;

TEST_CASE("DynamicBVH performance: Bulk insertion", "[DynamicBVH][Benchmark][Insert]")
{
#ifndef NDEBUG
    SKIP("Performance check skipped in Debug mode.");
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
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/10ms" << std::endl;

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
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/100ms" << std::endl;
}

TEST_CASE("DynamicBVH performance: Moving proxies", "[DynamicBVH][Benchmark][Move]")
{
#ifndef NDEBUG
    SKIP("Performance check skipped in Debug mode.");
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
    CHECK(elapsed < 50us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/50us" << std::endl;

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
    CHECK(elapsed < 500us);
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/500us" << std::endl;
}

TEST_CASE("DynamicBVH performance: Broad-phase AABB query", "[DynamicBVH][Benchmark][Query]")
{
#ifndef NDEBUG
    SKIP("Performance check skipped in Debug mode.");
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
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/10us" << std::endl;

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
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/100us" << std::endl;
}

TEST_CASE("DynamicBVH performance: Piercing raycast", "[DynamicBVH][Benchmark][Raycast]")
{
#ifndef NDEBUG
    SKIP("Performance check skipped in Debug mode.");
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
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/20us" << std::endl;

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
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/200us" << std::endl;
}

TEST_CASE("DynamicBVH: BroadPhaseCollisions benchmark (10k random proxies)", "[DynamicBVH][BroadPhaseCollisions][Benchmark]")
{
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
    std::cout << std::endl << duration_cast<microseconds>(elapsed).count() << "us/50us" << std::endl;

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
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/5ms" << std::endl;

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

    CHECK(elapsed < 80ms);
    std::cout << std::endl << duration_cast<milliseconds>(elapsed).count() << "ms/80ms" << std::endl;
}
