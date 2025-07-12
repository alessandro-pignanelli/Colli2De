#include <colli2de/internal/geometry/RaycastShapes.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <colli2de/internal/geometry/AABB.hpp>

namespace c2d
{

std::optional<std::pair<float, float>> raycast(const Circle& circle,
                                               Transform transform,
                                               Ray ray)
{
    const Vec2 center = transform.apply(circle.center);
    const Vec2 rayDirection = ray.p2 - ray.p1;
    const Vec2 fromStartToCenter = ray.p1 - center;

    const float directionLengthSq = rayDirection.dot(rayDirection);
    const float startProjection = fromStartToCenter.dot(rayDirection);
    const float startDistanceSq = fromStartToCenter.dot(fromStartToCenter) -
                                   circle.radius * circle.radius;

    if (directionLengthSq <= 0.0f)
    {
        if (startDistanceSq <= 0.0f)
            return std::make_pair(0.0f, 0.0f);
        return std::nullopt;
    }

    if (startDistanceSq > 0.0f && startProjection > 0.0f)
        return std::nullopt;

    const float discriminant = startProjection * startProjection -
                               directionLengthSq * startDistanceSq;
    if (discriminant < 0.0f)
        return std::nullopt;

    const float root = std::sqrt(discriminant);
    float entry = (-startProjection - root) / directionLengthSq;
    float exit = (-startProjection + root) / directionLengthSq;

    if (entry > exit)
        std::swap(entry, exit);

    if (entry > 1.0f || exit < 0.0f)
        return std::nullopt;

    entry = std::max(entry, 0.0f);
    exit = std::min(exit, 1.0f);
    if (entry > exit)
        return std::nullopt;

    return std::make_pair(entry, exit);
}

std::optional<std::pair<float, float>> raycast(const Circle& circle,
                                               Transform transform,
                                               InfiniteRay ray)
{
    const Vec2 center = transform.apply(circle.center);
    const Vec2 rayDirection = ray.direction;
    const Vec2 fromStartToCenter = ray.start - center;

    const float directionLengthSq = rayDirection.dot(rayDirection);
    const float startProjection = fromStartToCenter.dot(rayDirection);
    const float startDistanceSq = fromStartToCenter.dot(fromStartToCenter) -
                                   circle.radius * circle.radius;

    if (directionLengthSq <= 0.0f)
    {
        if (startDistanceSq <= 0.0f)
            return std::make_pair(0.0f, 0.0f);
        return std::nullopt;
    }

    if (startDistanceSq > 0.0f && startProjection > 0.0f)
        return std::nullopt;

    const float discriminant = startProjection * startProjection -
                               directionLengthSq * startDistanceSq;
    if (discriminant < 0.0f)
        return std::nullopt;

    const float root = std::sqrt(discriminant);
    float entry = (-startProjection - root) / directionLengthSq;
    float exit = (-startProjection + root) / directionLengthSq;

    if (entry > exit)
        std::swap(entry, exit);
    if (exit < 0.0f)
        return std::nullopt;

    entry = std::max(entry, 0.0f);
    return std::make_pair(entry, exit);
}

std::optional<std::pair<float, float>> raycast(const Segment& segment,
                                               Transform transform,
                                               Ray ray)
{
    const Vec2 point1 = transform.apply(segment.start);
    const Vec2 point2 = transform.apply(segment.end);
    const Vec2 segmentVector = point2 - point1;
    const Vec2 rayVector = ray.p2 - ray.p1;
    const Vec2 delta = ray.p1 - point1;

    const float crossSegRay = segmentVector.cross(rayVector);
    const float crossDeltaSeg = delta.cross(segmentVector);

    if (std::abs(crossSegRay) < 1e-8f)
        return std::nullopt;

    const float tSegment = delta.cross(rayVector) / crossSegRay;
    const float tRay = crossDeltaSeg / crossSegRay;

    if (tSegment >= 0.0f && tSegment <= 1.0f && tRay >= 0.0f && tRay <= 1.0f)
        return std::make_pair(tRay, tRay);

    return std::nullopt;
}

std::optional<std::pair<float, float>> raycast(const Segment& segment,
                                               Transform transform,
                                               InfiniteRay ray)
{
    const Vec2 point1 = transform.apply(segment.start);
    const Vec2 point2 = transform.apply(segment.end);
    const Vec2 segmentVector = point2 - point1;
    const Vec2 rayVector = ray.direction;
    const Vec2 delta = ray.start - point1;

    const float crossSegRay = segmentVector.cross(rayVector);
    const float crossDeltaSeg = delta.cross(segmentVector);

    if (std::abs(crossSegRay) < 1e-8f)
        return std::nullopt;

    const float tSegment = delta.cross(rayVector) / crossSegRay;
    const float tRay = crossDeltaSeg / crossSegRay;

    if (tSegment >= 0.0f && tSegment <= 1.0f && tRay >= 0.0f)
        return std::make_pair(tRay, tRay);

    return std::nullopt;
}

std::optional<std::pair<float, float>> raycast(const Polygon& polygon,
                                               Transform transform,
                                               Ray ray)
{
    const Vec2 rayDirection = ray.p2 - ray.p1;

    float lower = 0.0f;
    float upper = 1.0f;
    for (uint8_t i = 0; i < polygon.count; ++i)
    {
        const Vec2 vertex = transform.apply(polygon.vertices[i]);
        const Vec2 normal = transform.rotation.apply(polygon.normals[i]);
        const float numerator = normal.dot(vertex - ray.p1);
        const float denominator = normal.dot(rayDirection);

        if (std::abs(denominator) < 1e-8f)
        {
            if (numerator < 0.0f)
                return std::nullopt;
            continue;
        }

        const float t = numerator / denominator;
        if (denominator < 0.0f)
        {
            lower = std::max(lower, t);
        }
        else
        {
            upper = std::min(upper, t);
        }
        if (upper < lower)
            return std::nullopt;
    }

    return std::make_pair(lower, upper);
}

std::optional<std::pair<float, float>> raycast(const Polygon& polygon,
                                               Transform transform,
                                               InfiniteRay ray)
{
    const Vec2 rayDirection = ray.direction;

    float lower = 0.0f;
    float upper = std::numeric_limits<float>::max();
    for (uint8_t i = 0; i < polygon.count; ++i)
    {
        const Vec2 vertex = transform.apply(polygon.vertices[i]);
        const Vec2 normal = transform.rotation.apply(polygon.normals[i]);
        const float numerator = normal.dot(vertex - ray.start);
        const float denominator = normal.dot(rayDirection);

        if (std::abs(denominator) < 1e-8f)
        {
            if (numerator < 0.0f)
                return std::nullopt;
            continue;
        }

        const float t = numerator / denominator;
        if (denominator < 0.0f)
        {
            lower = std::max(lower, t);
        }
        else
        {
            upper = std::min(upper, t);
        }
        if (upper < lower)
            return std::nullopt;
    }

    return std::make_pair(lower, upper);
}

std::optional<std::pair<float, float>> raycast(const Capsule& capsule,
                                               Transform transform,
                                               Ray ray)
{
    const Vec2 worldStart = transform.apply(capsule.center1);
    const Vec2 worldEnd = transform.apply(capsule.center2);
    const float radius = capsule.radius;

    const Vec2 axis = worldEnd - worldStart;
    const float length = axis.length();
    if (length <= 1e-8f)
    {
        Circle circle{worldStart, radius};
        return raycast(circle, Transform{}, ray);
    }
    const Vec2 axisUnit = axis * (1.0f / length);
    const Vec2 side{ -axisUnit.y, axisUnit.x };

    const auto toLocal = [&](Vec2 point) -> Vec2
    {
        const Vec2 diff = point - worldStart;
        return Vec2{ diff.dot(axisUnit), diff.dot(side) };
    };

    Ray localRay{ toLocal(ray.p1), toLocal(ray.p2) };
    AABB core{ Vec2{0.0f, -radius}, Vec2{length, radius} };

    float entryTime = std::numeric_limits<float>::max();
    float exitTime = std::numeric_limits<float>::min();
    bool hit = false;

    auto update = [&](std::optional<std::pair<float, float>> iv)
    {
        if (!iv)
            return;
        hit = true;
        entryTime = std::min(entryTime, iv->first);
        exitTime = std::max(exitTime, iv->second);
    };

    update(core.intersects(localRay));

    Circle capA{ Vec2{0.0f, 0.0f}, radius };
    Circle capB{ Vec2{length, 0.0f}, radius };
    update(raycast(capA, Transform{}, localRay));
    update(raycast(capB, Transform{}, localRay));

    if (!hit)
        return std::nullopt;

    if (exitTime < 0.0f || entryTime > 1.0f || exitTime < entryTime)
        return std::nullopt;

    entryTime = std::max(entryTime, 0.0f);
    exitTime = std::min(exitTime, 1.0f);
    if (entryTime > exitTime)
        return std::nullopt;

    return std::make_pair(entryTime, exitTime);
}

std::optional<std::pair<float, float>> raycast(const Capsule& capsule,
                                               Transform transform,
                                               InfiniteRay ray)
{
    const Vec2 worldStart = transform.apply(capsule.center1);
    const Vec2 worldEnd = transform.apply(capsule.center2);
    const float radius = capsule.radius;

    const Vec2 axis = worldEnd - worldStart;
    const float length = axis.length();
    if (length <= 1e-8f)
    {
        Circle circle{worldStart, radius};
        return raycast(circle, Transform{}, ray);
    }
    const Vec2 axisUnit = axis * (1.0f / length);
    const Vec2 side{ -axisUnit.y, axisUnit.x };

    const auto toLocal = [&](Vec2 point) -> Vec2
    {
        const Vec2 diff = point - worldStart;
        return Vec2{ diff.dot(axisUnit), diff.dot(side) };
    };

    InfiniteRay localRay{ toLocal(ray.start),
                          Vec2{ ray.direction.dot(axisUnit),
                                ray.direction.dot(side) } };
    AABB core{ Vec2{0.0f, -radius}, Vec2{length, radius} };

    float entryTime = std::numeric_limits<float>::max();
    float exitTime = std::numeric_limits<float>::min();
    bool hit = false;

    auto update = [&](std::optional<std::pair<float, float>> iv)
    {
        if (!iv)
            return;
        hit = true;
        entryTime = std::min(entryTime, iv->first);
        exitTime = std::max(exitTime, iv->second);
    };

    update(core.intersects(localRay));

    Circle capA{ Vec2{0.0f, 0.0f}, radius };
    Circle capB{ Vec2{length, 0.0f}, radius };
    update(raycast(capA, Transform{}, localRay));
    update(raycast(capB, Transform{}, localRay));

    if (!hit)
        return std::nullopt;

    if (exitTime < 0.0f || entryTime > exitTime)
        return std::nullopt;

    entryTime = std::max(entryTime, 0.0f);
    return std::make_pair(entryTime, exitTime);
}

} // namespace c2d

