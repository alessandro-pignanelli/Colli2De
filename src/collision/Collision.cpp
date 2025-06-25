#include <algorithm>
#include <cmath> // For std::sqrt

#include "collision/Collision.hpp"

namespace c2d
{

Manifold collide(const Circle& circleA,
                 const Transform& transformA,
                 const Circle& circleB,
                 const Transform& transformB)
{
    Manifold manifold{};

    // Compute world-space centers
    const Vec2 centerA = transformA.apply(circleA.center);
    const Vec2 centerB = transformB.apply(circleB.center);

    // Vector from A to B
    const Vec2 delta = centerB - centerA;
    const float distSq = delta.x * delta.x + delta.y * delta.y;
    const float radiusA = circleA.radius;
    const float radiusB = circleB.radius;
    const float radius = radiusA + radiusB;
    constexpr float kEpsilon = 1e-6f;

    // Only collide if circles are closer than their combined radii (with a bit of speculation allowed)
    if (distSq > (radius + kEpsilon) * (radius + kEpsilon))
        return manifold; // No contact

    const float distance = std::sqrt(distSq);

    const Vec2 normal = (distance > kEpsilon) ? (delta * (1.0f / distance)) : Vec2{1.0f, 0.0f};
    const Vec2 contactA = centerA + normal * radiusA;
    const Vec2 contactB = centerB - normal * radiusB;
    const Vec2 worldContact = (contactA + contactB) * 0.5f;

    manifold.normal = normal;
    manifold.points[0].point = worldContact;
    manifold.points[0].anchorA = worldContact - transformA.translation;
    manifold.points[0].anchorB = worldContact - transformB.translation;
    manifold.points[0].separation = distance - radius; // Negative = penetration
    manifold.points[0].id = 0;
    manifold.pointCount = 1;

    return manifold;
}

Manifold collide(const Capsule& capsule,
                 const Transform& transformA,
                 const Circle& circle,
                 const Transform& transformB)
{
    Manifold manifold{};

    // Transform capsule endpoints to world coordinates
    const Vec2 center1 = transformA.apply(capsule.center1);
    const Vec2 center2 = transformA.apply(capsule.center2);
    const Vec2 centerCircle = transformB.apply(circle.center);

    // Find closest point on capsule segment to circle center
    const Vec2 segment = center2 - center1;
    const float segmentLengthSqr = segment.lengthSqr();
    float projection = 0.0f;
    if (segmentLengthSqr > 0.0f)
    {
        projection = (centerCircle - center1).dot(segment) / segmentLengthSqr;
        projection = std::clamp(projection, 0.0f, 1.0f);
    }
    const Vec2 closest = center1 + segment * projection;

    // Compute vector from capsule to circle center
    const Vec2 delta = centerCircle - closest;
    const float distanceSqr = delta.lengthSqr();
    const float sumRadius = capsule.radius + circle.radius;
    constexpr float kEpsilon = 1e-6f;

    if (distanceSqr > (sumRadius + kEpsilon) * (sumRadius + kEpsilon))
        return manifold; // No contact

    float distance = 0.0f;
    Vec2 normal{1.0f, 0.0f}; // Fallback axis
    if (distanceSqr > kEpsilon * kEpsilon)
    {
        distance = delta.length();
        normal = delta * (1.0f / distance);
    }

    // Find contact point (midpoint between surfaces)
    const Vec2 contactA = closest + normal * capsule.radius;
    const Vec2 contactB = centerCircle - normal * circle.radius;
    const Vec2 worldContact = (contactA + contactB) * 0.5f;

    manifold.normal = normal;
    manifold.points[0].point = worldContact;
    manifold.points[0].anchorA = worldContact - transformA.translation;
    manifold.points[0].anchorB = worldContact - transformB.translation;
    manifold.points[0].separation = distance - sumRadius; // Negative = penetration
    manifold.points[0].id = 0;
    manifold.pointCount = 1;

    return manifold;
}

Manifold collide(const Circle& circle,
                 const Transform& transformA,
                 const Capsule& capsule,
                 const Transform& transformB)
{
    auto manifold = collide(capsule, transformB, circle, transformA);

    // Flip the normal and anchors to match the circle's perspective
    manifold.normal = -manifold.normal;
    for (uint8_t i = 0; i < manifold.pointCount; ++i)
    {
        manifold.points[i].anchorA = manifold.points[i].anchorB;
        manifold.points[i].anchorB = manifold.points[i].anchorA;
    }

    return manifold;
}

} // namespace c2d