#include "geometry/AABB.hpp"

#include <algorithm>

namespace c2d
{

AABB AABB::fromCenter(Vec2 center, Vec2 halfSize)
{
    return AABB
    {
        center - halfSize,
        center + halfSize
    };
}

Vec2 AABB::center() const
{
    return (min + max) * 0.5f;
}

Vec2 AABB::size() const
{
    return max - min;
}

bool AABB::contains(Vec2 point) const
{
    return point.x >= min.x && point.x <= max.x &&
           point.y >= min.y && point.y <= max.y;
}

bool AABB::contains(AABB other) const
{
    return min.x <= other.min.x && min.y <= other.min.y &&
           max.x >= other.max.x && max.y >= other.max.y;
}

bool AABB::intersects(AABB other) const
{
    return min.x <= other.max.x && max.x >= other.min.x &&
           min.y <= other.max.y && max.y >= other.min.y;
}

AABB AABB::fattened(float margin) const
{
    return AABB
    {
        Vec2{ min.x - margin, min.y - margin },
        Vec2{ max.x + margin, max.y + margin }
    };
}

AABB AABB::fattened(float margin, Vec2 displacement) const
{
    // Grow min by negative displacement and margin
    AABB result = fattened(margin);

    if (displacement.x < 0.0f)
        result.min.x += displacement.x;
    else
        result.max.x += displacement.x;

    if (displacement.y < 0.0f)
        result.min.y += displacement.y;
    else
        result.max.y += displacement.y;
    
    return result;
}

AABB AABB::move(Vec2 direction) const
{
    return AABB
    {
        Vec2{ min.x + direction.x, min.y + direction.y },
        Vec2{ max.x + direction.x, max.y + direction.y }
    };
}

AABB AABB::expandToInclude(Vec2 point)
{
    return AABB
    {
        Vec2{ std::min(min.x, point.x), std::min(min.y, point.y) },
        Vec2{ std::max(max.x, point.x), std::max(max.y, point.y) }
    };
}

AABB AABB::combine(AABB a, AABB b)
{
    const auto minX = std::min(a.min.x, b.min.x);
    const auto minY = std::min(a.min.y, b.min.y);
    const auto maxX = std::max(a.max.x, b.max.x);
    const auto maxY = std::max(a.max.y, b.max.y);

    return AABB{ Vec2{minX, minY}, Vec2{maxX, maxY} };
}

float AABB::perimeter() const
{
    const float wx = max.x - min.x;
    const float wy = max.y - min.y;
    return 2.0f * (wx + wy);
}

} // namespace c2d
