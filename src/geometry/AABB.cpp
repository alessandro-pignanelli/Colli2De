#include "geometry/AABB.hpp"

#include <algorithm>
#include <utility>

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

std::optional<std::pair<float, float>> AABB::intersects(Ray ray) const
{
    // Slab method (compute the times when the ray enters and leaves the box "slab" along that axis)
    // Times are fractions of the total ray length
    // https://en.wikipedia.org/wiki/Slab_method
    float intersectionTimeMin = 0.0f;
    float intersectionTimeMax = 1.0f;
    Vec2 direction = ray.p2 - ray.p1;

    // X intersection
    {
        const float dir = direction.x;
        const float origin = ray.p1.x;
        if (std::abs(dir) < 1e-8f)
        {
            // Ray is parallel to X axis: only "inside" if origin is within [min.x, max.x]
            if (origin < min.x || origin > max.x)
                return std::nullopt;
        }
        else
        {
            const float invD = 1.0f / dir;
            float entryPoint = (min.x - origin) * invD;
            float exitPoint  = (max.x - origin) * invD;

            if (invD < 0.0f)
                std::swap(entryPoint, exitPoint);

            intersectionTimeMin = std::max(intersectionTimeMin, entryPoint);
            intersectionTimeMax = std::min(intersectionTimeMax, exitPoint);

            if (intersectionTimeMax < intersectionTimeMin)
                return std::nullopt;
        }
    }

    // Y intersection (same as X)
    {
        const float dir = direction.y;
        const float origin = ray.p1.y;
        if (std::abs(dir) < 1e-8f)
        {
            if (origin < min.y || origin > max.y)
                return std::nullopt;
        }
        else
        {
            const float invD = 1.0f / dir;
            float entryPoint = (min.y - origin) * invD;
            float exitPoint  = (max.y - origin) * invD;

            if (invD < 0.0f)
                std::swap(entryPoint, exitPoint);

            intersectionTimeMin = std::max(intersectionTimeMin, entryPoint);
            intersectionTimeMax = std::min(intersectionTimeMax, exitPoint);

            if (intersectionTimeMax < intersectionTimeMin)
                return std::nullopt;
        }
    }

    return std::make_pair(intersectionTimeMin, intersectionTimeMax);
}

std::optional<std::pair<float, float>> AABB::intersects(InfiniteRay ray) const
{
    float intersectionTimeMin = 0.0f;
    float intersectionTimeMax = std::numeric_limits<float>::max();

    // X intersection
    {
        const float dir = ray.direction.x;
        const float origin = ray.start.x;

        if (std::abs(dir) < 1e-8f) // Ray is (almost) parallel to X slabs
        {
            if (origin < min.x || origin > max.x)
                return std::nullopt; // Parallel and outside the box
        }
        else
        {
            float invD = 1.0f / dir;
            float entryPoint = (min.x - origin) * invD;
            float exitPoint  = (max.x - origin) * invD;

            if (invD < 0.0f)
                std::swap(entryPoint, exitPoint);

            intersectionTimeMin = std::max(intersectionTimeMin, entryPoint);
            intersectionTimeMax = std::min(intersectionTimeMax, exitPoint);

            if (intersectionTimeMax < intersectionTimeMin)
                return std::nullopt;
        }
    }

    // Y intersection
    {
        const float dir = ray.direction.y;
        const float origin = ray.start.y;

        if (std::abs(dir) < 1e-8f) // Ray is (almost) parallel to Y slabs
        {
            if (origin < min.y || origin > max.y)
                return std::nullopt; // Parallel and outside the box
        }
        else
        {
            const float invD = 1.0f / dir;
            float entryPoint = (min.y - origin) * invD;
            float exitPoint  = (max.y - origin) * invD;

            if (invD < 0.0f)
                std::swap(entryPoint, exitPoint);

            intersectionTimeMin = std::max(intersectionTimeMin, entryPoint);
            intersectionTimeMax = std::min(intersectionTimeMax, exitPoint);

            if (intersectionTimeMax < intersectionTimeMin)
                return std::nullopt;
        }
    }

    return std::make_pair(intersectionTimeMin, intersectionTimeMax);
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
