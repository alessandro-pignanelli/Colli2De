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
    bool contains(AABB other) const;
    bool intersects(AABB other) const;
    bool overlaps(AABB other) const;

    AABB fattened(float margin) const;
    AABB fattened(float margin, Vec2 displacement) const;
    AABB move(Vec2 direction) const;
    AABB expandToInclude(Vec2 point);
    float perimeter() const;

    bool operator==(const AABB& other) const
    {
        return min == other.min && max == other.max;
    }
};

static_assert(std::is_trivially_copyable_v<AABB>, "AABB must be trivially copyable");

}
