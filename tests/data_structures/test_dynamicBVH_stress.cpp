#include <cstdint>
#include <set>
#include <vector>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/interfaces/catch_interfaces_config.hpp>
#include <catch2/internal/catch_context.hpp>

#include "data_structures/DynamicBVH.hpp"
#include "geometry/AABB.hpp"
#include "utils/Performance.hpp"
#include "utils/Random.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("DynamicBVH can handle many proxies", "[DynamicBVH][Stress]")
{
    constexpr float margin = 0.01f;
    DynamicBVH<uint32_t> bvh(margin);

    const int N = 1000;
    for (int i = 0; i < N; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        bvh.createProxy({Vec2{x,0}, Vec2{x+0.05f,1}}, i);
    }

    // Query a range covering many
    AABB query{Vec2{10,0}, Vec2{20,1}};
    std::set<uint32_t> hits;
    bvh.query(query, hits);

    // Should find about (20-10)/0.1 = 100 proxies
    CHECK(hits.size() >= 90);
    CHECK(hits.size() <= 110);
}

TEST_CASE("DynamicBVH stays balanced with 100k proxies", "[DynamicBVH][Stress][Balance]")
{
    const auto seed = Catch::getCurrentContext().getConfig()->rngSeed();
    constexpr float regionMin = 0.0f, regionMax = 10'000.0f, size = 1.0f;

    // Deterministic random AABBs
    const std::vector<AABB> aabbs = generateRandomAABBs(100'000, 0.0f, 100.0f, 2.0f, seed);

    BENCHMARK_FUNCTION("BVH 100k Proxy creation and destruction", 300ms, [&]()
    {
        DynamicBVH<uint32_t> bvh;
        std::vector<uint32_t> ids;
        std::vector<NodeIndex> nodeIndices;

        const auto checkBalancing = [&]()
        {
            NodeIndex rootIndex = bvh.getRootIndex();
            REQUIRE(rootIndex != -1);
            
            // Tree height should be O(log2(N))
            const int32_t height = bvh.getNode(rootIndex).height;
            const int logN = static_cast<int>(std::ceil(std::log2(bvh.proxies())));
            
            // Allow a little extra for imperfect balancing and fattening
            // println("Tree height: {}, optimal = {}, expected <= {}", height, logN, logN + 3);
            CHECK(height <= logN + 3);
        };
        const auto checkTreeValidity = [&]()
        {
            std::vector<NodeIndex> stack = std::vector<NodeIndex>{bvh.getRootIndex()};
            std::set<uint32_t> foundIds;
            int leafCount = 0;

            while (!stack.empty())
            {
                NodeIndex idx = stack.back();
                stack.pop_back();
                if (idx == -1) continue;
                const auto& node = bvh.getNode(idx);

                if (node.isLeaf())
                {
                    ++leafCount;
                    CHECK(node.id < aabbs.size());
                    foundIds.insert(node.id);
                }
                else
                {
                    // Child points back to parent
                    CHECK(bvh.getNode(node.child1Index).parentIndex == idx);
                    CHECK(bvh.getNode(node.child2Index).parentIndex == idx);
                    stack.push_back(node.child1Index);
                    stack.push_back(node.child2Index);
                }
            }

            // All IDs are present and unique
            CHECK(leafCount == bvh.proxies());
            CHECK(foundIds.size() == bvh.proxies());
        };

        for (int i = 0; i < aabbs.size(); ++i)
        {
            const auto index = bvh.createProxy(aabbs[i], i);
            nodeIndices.push_back(index);
            ids.push_back(i);
        }
        CHECK(bvh.proxies() == aabbs.size());

        checkBalancing();
        checkTreeValidity();

        // Remove half the proxies
        for (size_t i = 0; i < nodeIndices.size(); i += 2)
            bvh.destroyProxy(nodeIndices[i]);
        CHECK(bvh.proxies() == aabbs.size() / 2);

        checkBalancing();
        checkTreeValidity();

        // Add back the removed proxies
        for (size_t i = 0; i < nodeIndices.size(); i += 2)
            bvh.createProxy(aabbs[i], ids[i]);
        CHECK(bvh.proxies() == aabbs.size());

        checkBalancing();
        checkTreeValidity();

        return bvh.proxies();
    });
}
