#include <cstdint>
#include <set>
#include <catch2/catch_test_macros.hpp>

#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("BroadPhaseTree | handle many proxies", "[BroadPhaseTree][Stress]")
{
    BroadPhaseTree<uint32_t> tree;
    const int N = 1000;
    for (int i = 0; i < N; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        tree.addProxy(i, {{x,0}, {x+0.05f,1}});
    }

    AABB query{Vec2{10,0}, Vec2{20,1}};
    auto hits = tree.query(query);
    CHECK(hits.size() >= 90);
    CHECK(hits.size() <= 110);
}
