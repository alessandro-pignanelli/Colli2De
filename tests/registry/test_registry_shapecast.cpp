#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "colli2de/Registry.hpp"

using namespace c2d;

TEST_CASE("Registry shapeCast detects sweep collision", "[Registry][ShapeCast]")
{
    Registry<int> reg;
    reg.createEntity(1, BodyType::Dynamic, {-2.0f, 0.0f});
    reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
    reg.createEntity(2, BodyType::Static, {1.0f, 0.0f});
    reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });

    auto hit = reg.shapeCast(1, {4.0f, 0.0f});
    REQUIRE(hit);
    CHECK(hit->id == 2);
    CHECK(hit->entryTime == Catch::Approx(0.25f).margin(0.01f));
}
