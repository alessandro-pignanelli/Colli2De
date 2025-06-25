#pragma once

#include <algorithm>

#include "colli2de/Shapes.hpp"
#include "geometry/AABB.hpp"
#include "geometry/Transformations.hpp"

namespace c2d
{

// --- Circle ---
inline AABB computeAABB(const Circle& circle, const Transform& transform)
{
    const Vec2 center = transform.apply(circle.center);
    const float radius = circle.radius;
    return AABB{center - radius, center + radius};
}

// --- Capsule ---
inline AABB computeAABB(const Capsule& capsule, const Transform& transform)
{
    const Vec2 p1 = transform.apply(capsule.center1);
    const Vec2 p2 = transform.apply(capsule.center2);
    const float radius = capsule.radius;

    const Vec2 minVertex{
        std::min(p1.x, p2.x) - radius,
        std::min(p1.y, p2.y) - radius
    };
    const Vec2 maxVertex{
        std::max(p1.x, p2.x) + radius,
        std::max(p1.y, p2.y) + radius
    };

    return AABB{minVertex, maxVertex};
}

// --- Segment ---
inline AABB computeAABB(const Segment& segment, const Transform& transform)
{
    const Vec2 p1 = transform.apply(segment.start);
    const Vec2 p2 = transform.apply(segment.end);

    const Vec2 minVertex{ std::min(p1.x, p2.x), std::min(p1.y, p2.y) };
    const Vec2 maxVertex{ std::max(p1.x, p2.x), std::max(p1.y, p2.y) };

    return AABB{minVertex, maxVertex};
}

// --- Polygon ---
inline AABB computeAABB(const Polygon& polygon, const Transform& transform)
{
    Vec2 minVertex = transform.apply(polygon.vertices[0]);
    Vec2 maxVertex = minVertex;
    for (uint8_t i = 1; i < polygon.count; ++i)
    {
        const Vec2 vertex = transform.apply(polygon.vertices[i]);
        minVertex.x = std::min(minVertex.x, vertex.x);
        minVertex.y = std::min(minVertex.y, vertex.y);
        maxVertex.x = std::max(maxVertex.x, vertex.x);
        maxVertex.y = std::max(maxVertex.y, vertex.y);
    }

    // Expand for polygon radius (rounded polygons)
    minVertex = minVertex - polygon.radius;
    maxVertex = maxVertex + polygon.radius;
    return AABB{minVertex, maxVertex};
}

} // namespace c2d