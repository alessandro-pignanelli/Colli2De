#pragma once

#include <functional>

#include <colli2de/Manifold.hpp>
#include <colli2de/Ray.hpp>
#include <colli2de/Shapes.hpp>
#include <colli2de/Transform.hpp>

namespace c2d
{

// --- Core collision manifold solvers ---
// All functions assume local shape definitions and a world transform for each shape.

// Circle vs Circle
Manifold collide(const Circle& circleA,
                 Transform transformA,
                 const Circle& circleB,
                 Transform transformB);

// Capsule vs Capsule
Manifold collide(const Capsule& capsuleA,
                 Transform transformA,
                 const Capsule& capsuleB,
                 Transform transformB);

// Polygon vs Polygon
Manifold collide(const Polygon& polygonA,
                 Transform transformA,
                 const Polygon& polygonB,
                 Transform transformB);

// Segment vs Segment
Manifold collide(const Segment& segmentA,
                 Transform transformA,
                 const Segment& segmentB,
                 Transform transformB);

// Circle vs Capsule
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB);
Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB);

// Circle vs Polygon
Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB);
Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB);

// Circle vs Segment
Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB);
Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB);

// Capsule vs Polygon
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB);
Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB);

// Capsule vs Segment
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB);
Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB);

// Polygon vs Segment
Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB);
Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB);

// --- Core collision boolean checks ---
bool areColliding(const Circle& circleA,
                  Transform transformA,
                  const Circle& circleB,
                  Transform transformB);
bool areColliding(const Capsule& capsule,
                  Transform transformA,
                  const Circle& circle,
                  Transform transformB);
bool areColliding(const Circle& circle,
                  Transform transformA,
                  const Capsule& capsule,
                  Transform transformB);
bool areColliding(const Capsule& capsuleA,
                  Transform transformA,
                  const Capsule& capsuleB,
                  Transform transformB);
bool areColliding(const Segment& segmentA,
                  Transform transformA,
                  const Segment& segmentB,
                  Transform transformB);
bool areColliding(const Circle& circle,
                  Transform transformA,
                  const Segment& segment,
                  Transform transformB);
bool areColliding(const Segment& segment,
                  Transform transformA,
                  const Circle& circle,
                  Transform transformB);
bool areColliding(const Capsule& capsule,
                  Transform transformA,
                  const Segment& segment,
                  Transform transformB);
bool areColliding(const Segment& segment,
                  Transform transformA,
                  const Capsule& capsule,
                  Transform transformB);
bool areColliding(const Polygon& polygonA,
                  Transform transformA,
                  const Polygon& polygonB,
                  Transform transformB);
bool areColliding(const Polygon& polygon,
                  Transform transformA,
                  const Circle& circle,
                  Transform transformB);
bool areColliding(const Circle& circle,
                  Transform transformA,
                  const Polygon& polygon,
                  Transform transformB);
bool areColliding(const Capsule& capsule,
                  Transform transformA,
                  const Polygon& polygon,
                  Transform transformB);
bool areColliding(const Polygon& polygon,
                  Transform transformA,
                  const Capsule& capsule,
                  Transform transformB);
bool areColliding(const Polygon& polygon,
                  Transform transformA,
                  const Segment& segment,
                  Transform transformB);
bool areColliding(const Segment& segment,
                  Transform transformA,
                  const Polygon& polygon,
                  Transform transformB);


// --- Sweep collision detection ---
inline std::optional<float> sweepHelper(Transform startTransform,
                                        Transform endTransform,
                                        const std::function<bool(const Transform&)>& isColliding)
{
    if (isColliding(startTransform))
        return 0.0f;

    const Transform deltaTransform = endTransform - startTransform;
    Transform currentTransform;

    constexpr int32_t coarseSteps = 8;
    constexpr int32_t refinementSteps = 10;
    float fractionLower = 0.0f;
    float fractionUpper = 1.0f;

    for (int32_t stepIndex = 1; stepIndex <= coarseSteps; ++stepIndex)
    {
        const float testFraction = static_cast<float>(stepIndex) / static_cast<float>(coarseSteps);
        currentTransform.translation = startTransform.translation + deltaTransform.translation * testFraction;
        currentTransform.rotation = startTransform.rotation.angleRadians + deltaTransform.rotation.angleRadians * testFraction;

        if (isColliding(currentTransform))
        {
            fractionLower = static_cast<float>(stepIndex - 1) / static_cast<float>(coarseSteps);
            fractionUpper = testFraction;
            break;
        }
    }
    
    // If no collision was found in the coarse steps, return no hit
    if (fractionLower == 0.0f && fractionUpper == 1.0f)
        return std::nullopt;

    for (int32_t iteration = 0; iteration < refinementSteps; ++iteration)
    {
        const float midFraction = 0.5f * (fractionLower + fractionUpper);
        currentTransform.translation = startTransform.translation + deltaTransform.translation * midFraction;
        currentTransform.rotation = startTransform.rotation.angleRadians + deltaTransform.rotation.angleRadians * midFraction;

        if (isColliding(currentTransform))
            fractionUpper = midFraction;
        else
            fractionLower = midFraction;

        // Stop if the precision is sufficient
        if (fractionUpper - fractionLower < 1e-6f)
            break;
    }

    return fractionUpper;
}

inline std::optional<float> sweepHelper(Transform startTransform1,
                                        Transform endTransform1,
                                        Transform startTransform2,
                                        Transform endTransform2,
                                        const std::function<bool(const Transform&, const Transform&)>& areColliding)
{
    if (areColliding(startTransform1, startTransform2))
        return 0.0f;

    const Transform deltaTransform1 = endTransform1 - startTransform1;
    const Transform deltaTransform2 = endTransform2 - startTransform2;
    Transform currentTransform1;
    Transform currentTransform2;

    constexpr int32_t coarseSteps = 8;
    constexpr int32_t refinementSteps = 10;
    float fractionLower = 0.0f;
    float fractionUpper = 1.0f;

    for (int32_t stepIndex = 1; stepIndex <= coarseSteps; ++stepIndex)
    {
        const float testFraction = static_cast<float>(stepIndex) / static_cast<float>(coarseSteps);

        currentTransform1.translation = startTransform1.translation + deltaTransform1.translation * testFraction;
        currentTransform1.rotation = startTransform1.rotation.angleRadians + deltaTransform1.rotation.angleRadians * testFraction;
        currentTransform2.translation = startTransform2.translation + deltaTransform2.translation * testFraction;
        currentTransform2.rotation = startTransform2.rotation.angleRadians + deltaTransform2.rotation.angleRadians * testFraction;

        if (areColliding(currentTransform1, currentTransform2))
        {
            fractionLower = static_cast<float>(stepIndex - 1) / static_cast<float>(coarseSteps);
            fractionUpper = testFraction;
            break;
        }
    }
    
    // If no collision was found in the coarse steps, return no hit
    if (fractionLower == 0.0f && fractionUpper == 1.0f)
        return std::nullopt;

    for (int32_t iteration = 0; iteration < refinementSteps; ++iteration)
    {
        const float midFraction = 0.5f * (fractionLower + fractionUpper);

        currentTransform1.translation = startTransform1.translation + deltaTransform1.translation * midFraction;
        currentTransform1.rotation = startTransform1.rotation.angleRadians + deltaTransform1.rotation.angleRadians * midFraction;
        currentTransform2.translation = startTransform2.translation + deltaTransform2.translation * midFraction;
        currentTransform2.rotation = startTransform2.rotation.angleRadians + deltaTransform2.rotation.angleRadians * midFraction;

        if (areColliding(currentTransform1, currentTransform2))
            fractionUpper = midFraction;
        else
            fractionLower = midFraction;

        // Stop if the precision is sufficient
        if (fractionUpper - fractionLower < 1e-6f)
            break;
    }

    return fractionUpper;
}

template <IsShape ShapeA, IsShape ShapeB>
std::optional<SweepManifold> sweep(const ShapeA& movingShape,
                                   Transform startTransform,
                                   Transform endTransform,
                                   const ShapeB& targetShape,
                                   Transform targetTransform)
{
    const auto isCollidingFunc = [&](const Transform& sweepTransform) -> bool
    {
        return areColliding(movingShape, sweepTransform, targetShape, targetTransform);
    };

    std::optional<float> collisionFraction = sweepHelper(startTransform,
                                                         endTransform,
                                                         isCollidingFunc);
    if (!collisionFraction)
        return std::nullopt;

    const Transform deltaTransform = endTransform - startTransform;
    Transform currentTransform {startTransform.translation + deltaTransform.translation * *collisionFraction,
                                startTransform.rotation.angleRadians + deltaTransform.rotation.angleRadians * *collisionFraction};
    return SweepManifold{*collisionFraction, collide(movingShape, currentTransform, targetShape, targetTransform)};
}

template <IsShape ShapeA, IsShape ShapeB>
std::optional<SweepManifold> sweep(const ShapeA& movingShape1,
                                   Transform startTransform1,
                                   Transform endTransform1,
                                   const ShapeB& movingShape2,
                                   Transform startTransform2,
                                   Transform endTransform2)
{
    const auto areCollidingFunc = [&](const Transform& transformShapeA, const Transform& transformShapeB) -> bool
    {
        return areColliding(movingShape1, transformShapeA, movingShape2, transformShapeB);
    };

    std::optional<float> collisionFraction = sweepHelper(startTransform1,
                                                         endTransform1,
                                                         startTransform2,
                                                         endTransform2,
                                                         areCollidingFunc);
    if (!collisionFraction)
        return std::nullopt;

    const Transform deltaTransform1 = endTransform1 - startTransform1;
    const Transform deltaTransform2 = endTransform2 - startTransform2;

    Transform targetTransform1 {startTransform1.translation + deltaTransform1.translation * *collisionFraction,
                                startTransform1.rotation.angleRadians + deltaTransform1.rotation.angleRadians * *collisionFraction};
    Transform targetTransform2 {startTransform2.translation + deltaTransform2.translation * *collisionFraction,
                                startTransform2.rotation.angleRadians + deltaTransform2.rotation.angleRadians * *collisionFraction};

    return SweepManifold{*collisionFraction, collide(movingShape1, targetTransform1, movingShape2, targetTransform2)};
}

template <IsShape ShapeA, IsRay RayType>
std::optional<std::pair<float, float>> sweep(const ShapeA& movingShape,
                                             Transform startTransform,
                                             Transform endTransform,
                                             RayType ray)
{
    std::optional<std::pair<float, float>> raycastResult = std::nullopt;
    const auto isCollidingFunc = [&](const Transform& sweepTransform) -> bool
    {
        const auto currentRaycastResult = raycast(movingShape, sweepTransform, ray);
        if (currentRaycastResult)
        {
            raycastResult = currentRaycastResult;
            return true;
        }
        return false;
    };

    sweepHelper(startTransform, endTransform, isCollidingFunc);
    return raycastResult;
}

} // namespace c2d
