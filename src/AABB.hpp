#pragma once

#include "colli2de/Vec2.hpp"

namespace c2d
{

struct AABB
{
    Vec2 min;
    Vec2 max;

    AABB() = default;
    AABB(Vec2 min, Vec2 max) : min{min}, max{max} {}

    static AABB fromCenter(Vec2 center, Vec2 halfSize);

    // Returns the smallest AABB that contains both a and b
    static AABB combine(AABB a, AABB b);

    Vec2 center() const;
    Vec2 size() const;

    bool contains(Vec2 point) const;
    bool intersects(AABB other) const;

    void expandToInclude(Vec2 point);
    float perimeter() const;
};

}
