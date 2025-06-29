#include <algorithm>
#include <cmath>
#include <vector>

#include "collision/Collision.hpp"

namespace c2d
{

namespace
{
    constexpr float kEpsilon = 1e-6f;
}

Manifold collide(const Circle& circleA,
                 Transform transformA,
                 const Circle& circleB,
                 Transform transformB)
{
    Manifold manifold{};

    // Compute world-space centers
    const Vec2 centerA = transformA.apply(circleA.center);
    const Vec2 centerB = transformB.apply(circleB.center);

    // Vector from A to B
    const Vec2 delta = centerB - centerA;
    const float distSq = delta.lengthSqr();
    const float radiusA = circleA.radius;
    const float radiusB = circleB.radius;
    const float radius = radiusA + radiusB;

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
    manifold.points[0].anchorA = transformA.toLocal(worldContact);
    manifold.points[0].anchorB = transformB.toLocal(worldContact);
    manifold.points[0].separation = distance - radius; // Negative = penetration
    manifold.points[0].id = 0;
    manifold.pointCount = 1;

    return manifold;
}

Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB)
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
    manifold.points[0].anchorA = transformA.toLocal(worldContact);
    manifold.points[0].anchorB = transformB.toLocal(worldContact);
    manifold.points[0].separation = distance - sumRadius; // Negative = penetration
    manifold.points[0].id = 0;
    manifold.pointCount = 1;

    return manifold;
}

Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB)
{
    auto manifold = collide(capsule, transformB, circle, transformA);
    manifold.reverse();
    return manifold;
}

// --- Capsule vs Capsule ---
Manifold collide(const Capsule& capsuleA,
                 Transform transformA,
                 const Capsule& capsuleB,
                 Transform transformB)
{
    Manifold manifold{};

    const Vec2 center1A = transformA.apply(capsuleA.center1);
    const Vec2 center2A = transformA.apply(capsuleA.center2);
    const Vec2 center1B = transformB.apply(capsuleB.center1);
    const Vec2 center2B = transformB.apply(capsuleB.center2);

    const Vec2 segmentVectorA = center2A - center1A;
    const Vec2 segmentVectorB = center2B - center1B;
    const Vec2 startDelta = center1A - center1B;

    const float lengthSquaredA = segmentVectorA.dot(segmentVectorA);
    const float lengthSquaredB = segmentVectorB.dot(segmentVectorB);
    const float deltaDotB = segmentVectorB.dot(startDelta);

    float paramA = 0.0f;
    float paramB = 0.0f;

    if (lengthSquaredA <= kEpsilon && lengthSquaredB <= kEpsilon)
    {
        // Both degenerate to points
        paramA = paramB = 0.0f;
    }
    else if (lengthSquaredA <= kEpsilon)
    {
        paramA = 0.0f;
        paramB = std::clamp(deltaDotB / lengthSquaredB, 0.0f, 1.0f);
    }
    else
    {
        const float deltaDotA = segmentVectorA.dot(startDelta);
        if (lengthSquaredB <= kEpsilon)
        {
            paramB = 0.0f;
            paramA = std::clamp(-deltaDotA / lengthSquaredA, 0.0f, 1.0f);
        }
        else
        {
            const float segmentDotAB = segmentVectorA.dot(segmentVectorB);
            const float denominator = lengthSquaredA * lengthSquaredB - segmentDotAB * segmentDotAB;
            if (denominator != 0.0f)
            {
                paramA = std::clamp((segmentDotAB * deltaDotB - deltaDotA * lengthSquaredB) / denominator,
                                    0.0f,
                                    1.0f);
            }
            else
            {
                paramA = 0.0f;
            }
            float tNumerator = segmentDotAB * paramA + deltaDotB;
            if (tNumerator < 0.0f)
            {
                paramB = 0.0f;
                paramA = std::clamp(-deltaDotA / lengthSquaredA, 0.0f, 1.0f);
            }
            else if (tNumerator > lengthSquaredB)
            {
                paramB = 1.0f;
                paramA = std::clamp((segmentDotAB - deltaDotA) / lengthSquaredA, 0.0f, 1.0f);
            }
            else
            {
                paramB = tNumerator / lengthSquaredB;
            }
        }
    }

    const Vec2 pointA = center1A + segmentVectorA * paramA;
    const Vec2 pointB = center1B + segmentVectorB * paramB;

    const Vec2 delta = pointB - pointA;
    const float distSq = delta.lengthSqr();
    const float radius = capsuleA.radius + capsuleB.radius;

    if (distSq > (radius + kEpsilon) * (radius + kEpsilon))
        return manifold;

    float distance = 0.0f;
    Vec2 normal{1.0f, 0.0f};
    if (distSq > kEpsilon * kEpsilon)
    {
        distance = std::sqrt(distSq);
        normal = delta * (1.0f / distance);
    }

    const Vec2 contactA = pointA + normal * capsuleA.radius;
    const Vec2 contactB = pointB - normal * capsuleB.radius;
    const Vec2 worldContact = (contactA + contactB) * 0.5f;

    manifold.normal = normal;
    manifold.points[0].point = worldContact;
    manifold.points[0].anchorA = transformA.toLocal(worldContact);
    manifold.points[0].anchorB = transformB.toLocal(worldContact);
    manifold.points[0].separation = distance - radius;
    manifold.points[0].id = 0;
    manifold.pointCount = 1;

    return manifold;
}

// --- Segment vs Segment ---
Manifold collide(const Segment& segmentA,
                 Transform transformA,
                 const Segment& segmentB,
                 Transform transformB)
{
    Capsule capA{ segmentA.start, segmentA.end, 0.0f };
    Capsule capB{ segmentB.start, segmentB.end, 0.0f };
    return collide(capA, transformA, capB, transformB);
}

// --- Circle vs Segment ---
Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB)
{
    Capsule cap{ segment.start, segment.end, 0.0f };
    auto manifold = collide(cap, transformB, circle, transformA);
    manifold.reverse();
    return manifold;
}

Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB)
{
    Capsule cap{ segment.start, segment.end, 0.0f };
    return collide(cap, transformA, circle, transformB);
}

// --- Capsule vs Segment ---
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB)
{
    Capsule segCap{ segment.start, segment.end, 0.0f };
    return collide(capsule, transformA, segCap, transformB);
}

Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB)
{
    Capsule segCap{ segment.start, segment.end, 0.0f };
    auto manifold = collide(capsule, transformB, segCap, transformA);
    manifold.reverse();
    return manifold;
}

// Helper for polygon SAT projection
static std::pair<float, float> projectPolygon(Polygon polygon,
                                              Transform transform,
                                              Vec2 axis)
{
    const Vec2 first = transform.apply(polygon.vertices[0]);
    float outMin = axis.dot(first);
    float outMax = outMin;
    for (uint8_t i = 1; i < polygon.count; ++i)
    {
        const float proj = axis.dot(transform.apply(polygon.vertices[i]));
        outMin = std::min(outMin, proj);
        outMax = std::max(outMax, proj);
    }
    return { outMin, outMax };
}

static std::vector<Vec2> transformVertices(Polygon polygon, Transform transform)
{
    std::vector<Vec2> verts;
    verts.reserve(polygon.count);
    for (uint8_t i = 0; i < polygon.count; ++i)
    {
        verts.push_back(transform.apply(polygon.vertices[i]));
    }
    return verts;
}

static std::vector<Vec2> clipWithPlane(const std::vector<Vec2>& input, Vec2 point, Vec2 normal)
{
    std::vector<Vec2> output;
    if (input.empty())
        return output;

    Vec2 prev = input.back();
    const float prevDist = normal.dot(prev - point);
    float previousDistance = prevDist;
    for (const Vec2& curr : input)
    {
        const float currentDistance = normal.dot(curr - point);
        if (currentDistance <= 0.0f)
        {
            if (previousDistance > 0.0f)
            {
                const float interpolationFactor = previousDistance / (previousDistance - currentDistance);
                const Vec2 intersect = prev + (curr - prev) * interpolationFactor;
                output.push_back(intersect);
            }
            output.push_back(curr);
        }
        else if (previousDistance <= 0.0f)
        {
            const float interpolationFactor = previousDistance / (previousDistance - currentDistance);
            const Vec2 intersect = prev + (curr - prev) * interpolationFactor;
            output.push_back(intersect);
        }
        prev = curr;
        previousDistance = currentDistance;
    }
    return output;
}

static std::vector<Vec2> polygonIntersection(Polygon polygonA,
                                             Transform transformA,
                                             Polygon polygonB,
                                             Transform transformB)
{
    std::vector<Vec2> result = transformVertices(polygonA, transformA);
    std::vector<Vec2> clipVerts = transformVertices(polygonB, transformB);
    for (uint8_t i = 0; i < polygonB.count && !result.empty(); ++i)
    {
        const Vec2 planePoint = clipVerts[i];
        const Vec2 planeNormal = transformB.rotation.apply(polygonB.normals[i]);
        result = clipWithPlane(result, planePoint, planeNormal);
    }
    return result;
}

// --- Polygon vs Polygon ---
Manifold collide(const Polygon& polygonA,
                 Transform transformA,
                 const Polygon& polygonB,
                 Transform transformB)
{
    Manifold manifold{};

    float overlap = std::numeric_limits<float>::max();
    Vec2 smallestAxis{ 1.0f, 0.0f };

    const auto areAxesOverlapping = [&](Polygon polygon, Transform transform)
    {
        for (uint8_t i = 0; i < polygon.count; ++i)
        {
            const Vec2 axis = transform.rotation.apply(polygon.normals[i]);
            auto [minA, maxA] = projectPolygon(polygonA, transformA, axis);
            auto [minB, maxB] = projectPolygon(polygonB, transformB, axis);
            const float axisOverlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (axisOverlap < 0.0f)
                return false;
            if (axisOverlap < overlap)
            {
                overlap = axisOverlap;
                smallestAxis = axis;
            }
        }
        return true;
    };

    if (!areAxesOverlapping(polygonA, transformA))
        return manifold;
    if (!areAxesOverlapping(polygonB, transformB))
        return manifold;

    const Vec2 centerA = transformA.apply(polygonA.vertices[0]);
    Vec2 avgA = centerA;
    for (uint8_t i = 1; i < polygonA.count; ++i)
        avgA += transformA.apply(polygonA.vertices[i]);
    avgA = avgA / static_cast<float>(polygonA.count);

    const Vec2 centerB = transformB.apply(polygonB.vertices[0]);
    Vec2 avgB = centerB;
    for (uint8_t i = 1; i < polygonB.count; ++i)
        avgB += transformB.apply(polygonB.vertices[i]);
    avgB = avgB / static_cast<float>(polygonB.count);

    if ((avgB - avgA).dot(smallestAxis) < 0.0f)
        smallestAxis = -smallestAxis;

    std::vector<Vec2> contactVertices = polygonIntersection(polygonA, transformA, polygonB, transformB);
    manifold.normal = smallestAxis;

    if (contactVertices.empty())
    {
        const Vec2 worldContact = (avgA + avgB) * 0.5f;
        manifold.points[0].point = worldContact;
        manifold.points[0].anchorA = transformA.toLocal(worldContact);
        manifold.points[0].anchorB = transformB.toLocal(worldContact);
        manifold.points[0].separation = -overlap;
        manifold.points[0].id = 0;
        manifold.pointCount = 1;
        return manifold;
    }

    std::sort(contactVertices.begin(), contactVertices.end());

    const std::size_t count = std::min<std::size_t>(contactVertices.size(), MaxManifoldPoints);
    for (std::size_t i = 0; i < count; ++i)
    {
        const Vec2 contactPoint = contactVertices[i];
        manifold.points[i].point = contactPoint;
        manifold.points[i].anchorA = transformA.toLocal(contactPoint);
        manifold.points[i].anchorB = transformB.toLocal(contactPoint);
        manifold.points[i].separation = -overlap;
        manifold.points[i].id = static_cast<uint16_t>(i);
    }
    manifold.pointCount = static_cast<uint8_t>(count);

    return manifold;
}

// --- Circle vs Polygon ---
static bool isPointInsidePolygon(Vec2 point,
                                 Polygon polygon,
                                 Transform transform)
{
    for (uint8_t i = 0; i < polygon.count; ++i)
    {
        const Vec2 v1 = transform.apply(polygon.vertices[i]);
        const Vec2 v2 = transform.apply(polygon.vertices[(i + 1) % polygon.count]);
        const Vec2 edge = v2 - v1;
        const Vec2 normal = transform.rotation.apply(polygon.normals[i]);
        if (normal.dot(point - v1) > 0.0f)
            return false;
    }
    return true;
}

Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB)
{
    Manifold manifold{};

    const Vec2 center = transformB.apply(circle.center);
    const float combinedRadius = circle.radius + polygon.radius;

    float minDistSq = std::numeric_limits<float>::max();
    Vec2 closest{ 0.0f, 0.0f };
    Vec2 normal{ 1.0f, 0.0f };

    for (uint8_t i = 0; i < polygon.count; ++i)
    {
        const Vec2 v1 = transformA.apply(polygon.vertices[i]);
        const Vec2 v2 = transformA.apply(polygon.vertices[(i + 1) % polygon.count]);
        const Vec2 edge = v2 - v1;
        const float edgeLenSq = edge.lengthSqr();
        float edgeParam = 0.0f;
        if (edgeLenSq > 0.0f)
            edgeParam = std::clamp((center - v1).dot(edge) / edgeLenSq, 0.0f, 1.0f);
        const Vec2 pointOnEdge = v1 + edge * edgeParam;
        const Vec2 delta = center - pointOnEdge;
        const float distSq = delta.lengthSqr();
        if (distSq < minDistSq)
        {
            minDistSq = distSq;
            closest = pointOnEdge;
            if (edgeParam > 0.0f && edgeParam < 1.0f)
                normal = transformA.rotation.apply(polygon.normals[i]);
            else
                normal = delta.lengthSqr() > kEpsilon * kEpsilon ? delta.normalize() : transformA.rotation.apply(polygon.normals[i]);
        }
    }

    const bool isInside = isPointInsidePolygon(center, polygon, transformA);
    const float radius = combinedRadius;

    if (!isInside && minDistSq > radius * radius)
        return manifold;

    const float distance = std::sqrt(minDistSq);
    if (!isInside && distance > kEpsilon)
        normal = (center - closest) * (1.0f / distance);
    else if (isInside)
        normal = -normal;

    const Vec2 contactA = closest;
    const Vec2 contactB = center - normal * circle.radius;
    const Vec2 worldContact = (contactA + contactB) * 0.5f;

    manifold.normal = normal;
    manifold.points[0].point = worldContact;
    manifold.points[0].anchorA = transformA.toLocal(worldContact);
    manifold.points[0].anchorB = transformB.toLocal(worldContact);
    if (isInside)
        manifold.points[0].separation = -(circle.radius + distance);
    else
        manifold.points[0].separation = distance - radius;
    manifold.points[0].id = 0;
    manifold.pointCount = 1;

    return manifold;
}

Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB)
{
    auto manifold = collide(polygon, transformB, circle, transformA);
    manifold.reverse();
    return manifold;
}

// --- Capsule vs Polygon ---
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB)
{
    const Vec2 axis = (capsule.center2 - capsule.center1).normalize();
    const Vec2 normal = Vec2{ -axis.y, axis.x };

    Polygon capsulePoly{};
    capsulePoly.vertices[0] = capsule.center1;
    capsulePoly.vertices[1] = capsule.center2;
    capsulePoly.normals[0] = normal;
    capsulePoly.normals[1] = -normal;
    capsulePoly.count = 2;
    capsulePoly.radius = capsule.radius;

    return collide(capsulePoly, transformA, polygon, transformB);
}

Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB)
{
    auto manifold = collide(capsule, transformB, polygon, transformA);
    manifold.reverse();
    return manifold;
}

// --- Polygon vs Segment ---
Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB)
{
    Capsule segCap{ segment.start, segment.end, 0.0f };
    return collide(polygon, transformA, segCap, transformB);
}

Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB)
{
    auto manifold = collide(polygon, transformB, segment, transformA);
    manifold.reverse();
    return manifold;
}

} // namespace c2d
