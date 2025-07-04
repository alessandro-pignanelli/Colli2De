#include <cstdint>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("BroadPhaseTree | proxies can be added and queried", "[BroadPhaseTree]")
{
    BroadPhaseTree<uint32_t> tree;

    auto h1 = tree.addProxy(1, {{0,0}, {1,1}});
    auto h2 = tree.addProxy(2, {{2,2}, {3,3}});

    REQUIRE(tree.size() == 2);

    auto hits = tree.query({{0,0}, {3,3}});
    CHECK(hits.count(1) == 1);
    CHECK(hits.count(2) == 1);

    tree.removeProxy(h1);
    hits = tree.query({{0,0}, {3,3}});
    CHECK(hits.count(1) == 0);
    CHECK(hits.count(2) == 1);
    REQUIRE(tree.size() == 1);
}

TEST_CASE("BroadPhaseTree | moveProxy updates location", "[BroadPhaseTree]")
{
    BroadPhaseTree<uint32_t> tree;
    auto h = tree.addProxy(1, {{0,0}, {1,1}});

    tree.moveProxy(h, {{5,5}, {6,6}});
    auto hits = tree.query({{5,5}, {6,6}});
    CHECK(hits.count(1) == 1);
    hits = tree.query({{0,0}, {1,1}});
    CHECK(hits.empty());
}

TEST_CASE("BroadPhaseTree | findAllCollisions detects overlaps", "[BroadPhaseTree]")
{
    BroadPhaseTree<uint32_t> tree;
    tree.addProxy(0, {{0,0}, {2,2}});
    tree.addProxy(1, {{1,1}, {3,3}});
    tree.addProxy(2, {{4,4}, {5,5}});
    tree.addProxy(3, {{1.5f,1.5f}, {2.5f,2.5f}});

    std::set<std::pair<uint32_t,uint32_t>> pairs = tree.findAllCollisions();
    std::set<std::pair<uint32_t,uint32_t>> expected{{0,1},{0,3},{1,3}};
    REQUIRE(pairs.size() == expected.size());
    for(const auto& p : expected)
        CHECK(pairs.count(p) == 1);
}
