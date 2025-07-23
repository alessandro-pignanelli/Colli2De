#include <cstdint>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <colli2de/internal/data_structures/BroadPhaseTree.hpp>
#include <colli2de/internal/geometry/AABB.hpp>

using namespace c2d;
using namespace Catch;

TEST_CASE("BroadPhaseTree | proxies can be added and queried", "[BroadPhaseTree]")
{
    BroadPhaseTree<uint32_t> tree;

    auto h1 = tree.addProxy(1, {{0,0}, {1,1}});
    auto h2 = tree.addProxy(2, {{2,2}, {3,3}});

    REQUIRE(tree.size() == 2);

    std::vector<uint32_t> hits;
    tree.query({{0,0}, {3,3}}, hits);
    CHECK(std::find(hits.begin(), hits.end(), 1) != hits.end());
    CHECK(std::find(hits.begin(), hits.end(), 2) != hits.end());

    hits.clear();

    tree.removeProxy(h1);
    tree.query({{0,0}, {3,3}}, hits);
    CHECK(std::find(hits.begin(), hits.end(), 1) == hits.end());
    CHECK(std::find(hits.begin(), hits.end(), 2) != hits.end());
    REQUIRE(tree.size() == 1);
}

TEST_CASE("BroadPhaseTree | moveProxy updates location", "[BroadPhaseTree]")
{
    BroadPhaseTree<uint32_t> tree;
    std::vector<uint32_t> hits;
    auto h = tree.addProxy(1, {{0,0}, {1,1}});

    tree.moveProxy(h, {{5,5}, {6,6}});
    tree.query({{5,5}, {6,6}}, hits);
    CHECK(std::find(hits.begin(), hits.end(), 1) != hits.end());

    hits.clear();
    tree.query({{0,0}, {1,1}}, hits);
    CHECK(hits.empty());
}

TEST_CASE("BroadPhaseTree | findAllCollisions detects overlaps", "[BroadPhaseTree]")
{
    BroadPhaseTree<uint32_t> tree;
    tree.addProxy(0, {{0,0}, {2,2}});
    tree.addProxy(1, {{1,1}, {3,3}});
    tree.addProxy(2, {{4,4}, {5,5}});
    tree.addProxy(3, {{1.5f,1.5f}, {2.5f,2.5f}});

    std::vector<std::pair<uint32_t, uint32_t>> pairs;
    tree.findAllCollisions([&pairs](uint32_t id1, uint32_t id2) {
        pairs.emplace_back(id1, id2);
    });
    std::vector<std::pair<uint32_t, uint32_t>> expected{{0,1},{0,3},{1,3}};
    REQUIRE(pairs.size() == expected.size());
    for(const auto& p : expected)
        CHECK(std::find(pairs.begin(), pairs.end(), p) != pairs.end());
}
