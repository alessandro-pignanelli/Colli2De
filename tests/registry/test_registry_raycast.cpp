#include <catch2/catch_test_macros.hpp>
#include "colli2de/Registry.hpp"

using namespace c2d;

TEST_CASE("Registry rayCast hits nearest entity", "[Registry][Raycast]")
{
    Registry<int> reg;
    reg.createEntity(1, BodyType::Static, Transform({0.0f, 0.0f}));
    reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
    reg.createEntity(2, BodyType::Static, Transform({3.0f, 0.0f}));
    reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });

    Ray ray{ {-2.0f, 0.0f}, {1.9f, 0.0f} };

    auto hits = reg.rayCast(ray);
    REQUIRE(hits.size() == 1);
    CHECK(hits.begin()->id.first == 1);

    auto hit = reg.firstHitRayCast(ray);
    REQUIRE(hit.has_value());
    CHECK(hit->id.first == 1);

    Ray ray2{ {4.0f, 0.0f}, {-2.0f, 0.0f} };

    hits = reg.rayCast(ray2);
    REQUIRE(hits.size() == 2);
    CHECK(hits.begin()->id.first == 2);
    CHECK(std::next(hits.begin())->id.first == 1);

    hit = reg.firstHitRayCast(ray2);
    REQUIRE(hit.has_value());
    CHECK(hit->id.first == 2);
}
