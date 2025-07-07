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

// --- Point containment ---
inline bool containsPoint(const Circle& circle, Transform transform, Vec2 point)
{
    const Vec2 center = transform.apply(circle.center);
    const Vec2 delta = point - center;
    return delta.lengthSqr() <= circle.radius * circle.radius;
}

inline bool containsPoint(const Capsule& capsule, Transform transform, Vec2 point)
{
    const Vec2 worldStart = transform.apply(capsule.center1);
    const Vec2 worldEnd = transform.apply(capsule.center2);
    const Vec2 segment = worldEnd - worldStart;
    const float lengthSqr = segment.lengthSqr();
    float projection = 0.0f;
    if (lengthSqr > 0.0f)
        projection = std::clamp((point - worldStart).dot(segment) / lengthSqr, 0.0f, 1.0f);
    const Vec2 closest = worldStart + segment * projection;
    const Vec2 delta = point - closest;
    return delta.lengthSqr() <= capsule.radius * capsule.radius;
}

inline bool containsPoint(const Segment& segment, Transform transform, Vec2 point)
{
    const Vec2 worldStart = transform.apply(segment.start);
    const Vec2 worldEnd = transform.apply(segment.end);
    const Vec2 edge = worldEnd - worldStart;
    const float lengthSqr = edge.lengthSqr();
    if (lengthSqr == 0.0f)
        return (point - worldStart).lengthSqr() <= 1e-6f;
    const float t = std::clamp((point - worldStart).dot(edge) / lengthSqr, 0.0f, 1.0f);
    const Vec2 closest = worldStart + edge * t;
    return (point - closest).lengthSqr() <= 1e-6f;
}

inline bool containsPoint(const Polygon& polygon, Transform transform, Vec2 point)
{
    for (uint8_t i = 0; i < polygon.count; ++i)
    {
        const Vec2 v1 = transform.apply(polygon.vertices[i]);
        const Vec2 normal = transform.rotation.apply(polygon.normals[i]);
        if (normal.dot(point - v1) > 0.0f)
            return false;
    }
    return true;
}

// --- Closest point ---
inline Vec2 closestPoint(const Circle& circle, Transform transform, Vec2 point)
{
    const Vec2 center = transform.apply(circle.center);
    const Vec2 delta = point - center;
    const float distSqr = delta.lengthSqr();
    if (distSqr <= circle.radius * circle.radius || distSqr == 0.0f)
        return point;
    const float dist = std::sqrt(distSqr);
    return center + delta * (circle.radius / dist);
}

inline Vec2 closestPoint(const Capsule& capsule, Transform transform, Vec2 point)
{
    const Vec2 worldStart = transform.apply(capsule.center1);
    const Vec2 worldEnd = transform.apply(capsule.center2);
    const Vec2 segment = worldEnd - worldStart;
    const float lengthSqr = segment.lengthSqr();
    float projection = 0.0f;
    if (lengthSqr > 0.0f)
        projection = std::clamp((point - worldStart).dot(segment) / lengthSqr, 0.0f, 1.0f);
    const Vec2 closestOnSegment = worldStart + segment * projection;
    const Vec2 delta = point - closestOnSegment;
    const float distSqr = delta.lengthSqr();
    if (distSqr <= capsule.radius * capsule.radius || distSqr == 0.0f)
        return point;
    const float dist = std::sqrt(distSqr);
    return closestOnSegment + delta * (capsule.radius / dist);
}

inline Vec2 closestPoint(const Segment& segment, Transform transform, Vec2 point)
{
    const Vec2 worldStart = transform.apply(segment.start);
    const Vec2 worldEnd = transform.apply(segment.end);
    const Vec2 edge = worldEnd - worldStart;
    const float lengthSqr = edge.lengthSqr();
    if (lengthSqr == 0.0f)
        return worldStart;
    const float t = std::clamp((point - worldStart).dot(edge) / lengthSqr, 0.0f, 1.0f);
    return worldStart + edge * t;
}

inline Vec2 closestPoint(const Polygon& polygon, Transform transform, Vec2 point)
{
    float minDistSqr = std::numeric_limits<float>::max();
    Vec2 closest{0.0f, 0.0f};
    bool inside = true;
    for (uint8_t i = 0; i < polygon.count; ++i)
    {
        const Vec2 v1 = transform.apply(polygon.vertices[i]);
        const Vec2 v2 = transform.apply(polygon.vertices[(i + 1) % polygon.count]);
        const Vec2 edge = v2 - v1;
        const float edgeLenSqr = edge.lengthSqr();
        float t = 0.0f;
        if (edgeLenSqr > 0.0f)
            t = std::clamp((point - v1).dot(edge) / edgeLenSqr, 0.0f, 1.0f);
        const Vec2 candidate = v1 + edge * t;
        const float distSqr = (point - candidate).lengthSqr();
        if (distSqr < minDistSqr)
        {
            minDistSqr = distSqr;
            closest = candidate;
        }
        const Vec2 normal = transform.rotation.apply(polygon.normals[i]);
        if (normal.dot(point - v1) > 0.0f)
            inside = false;
    }
    if (inside)
        return point;
    return closest;
}

// --- Minimum distance between shapes ---
namespace
{
template <typename Shape, typename Visitor>
inline void forEachVertex(const Shape& shape, const Visitor& visitor)
{
    for (uint8_t i = 0; i < shape.count; ++i)
        visitor(shape.vertices[i]);
}

template <typename Visitor>
inline void forEachVertex(const Circle& circle, const Visitor& visitor)
{
    visitor(circle.center);
}

template <typename Visitor>
inline void forEachVertex(const Capsule& capsule, const Visitor& visitor)
{
    visitor(capsule.center1);
    visitor(capsule.center2);
}

template <typename Visitor>
inline void forEachVertex(const Segment& segment, const Visitor& visitor)
{
    visitor(segment.start);
    visitor(segment.end);
}
} // anonymous namespace

template <typename ShapeA, typename ShapeB>
inline float distance(const ShapeA& shapeA,
                      Transform transformA,
                      const ShapeB& shapeB,
                      Transform transformB)
{
    float bestDistSqr = std::numeric_limits<float>::max();

    auto checkPointA = [&](Vec2 localVertex)
    {
        const Vec2 worldPoint = transformA.apply(localVertex);
        const Vec2 pointOnB = closestPoint(shapeB, transformB, worldPoint);
        const Vec2 pointOnA = closestPoint(shapeA, transformA, pointOnB);
        const float distSqr = (pointOnA - pointOnB).lengthSqr();
        bestDistSqr = std::min(bestDistSqr, distSqr);
    };
    forEachVertex(shapeA, checkPointA);

    auto checkPointB = [&](Vec2 localVertex)
    {
        const Vec2 worldPoint = transformB.apply(localVertex);
        const Vec2 pointOnA = closestPoint(shapeA, transformA, worldPoint);
        const Vec2 pointOnB = closestPoint(shapeB, transformB, pointOnA);
        const float distSqr = (pointOnA - pointOnB).lengthSqr();
        bestDistSqr = std::min(bestDistSqr, distSqr);
    };
    forEachVertex(shapeB, checkPointB);

    return std::sqrt(bestDistSqr);
}


} // namespace c2d
