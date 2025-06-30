#pragma once

#include "colli2de/Shapes.hpp"
#include "collision/Manifold.hpp"
#include "geometry/Transformations.hpp"

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


// --- Sweep collision detection ---
template <typename ShapeA, typename ShapeB>
std::optional<SweepManifold> sweep(const ShapeA& movingShape,
                              Transform startTransform,
                              Vec2 translation,
                              Rotation endRotation,
                              const ShapeB& targetShape,
                              Transform targetTransform)
{
    const float startAngle = std::atan2(startTransform.rotation.sin, startTransform.rotation.cos);
    const float endAngle = std::atan2(endRotation.sin, endRotation.cos);

    auto manifoldAt = [&](float fraction) -> Manifold
    {
        Transform currentTransform;
        currentTransform.translation = startTransform.translation + translation * fraction;
        const float currentAngle = startAngle + (endAngle - startAngle) * fraction;
        currentTransform.rotation = Rotation{currentAngle};
        return collide(movingShape, currentTransform, targetShape, targetTransform);
    };

    Manifold initialManifold = manifoldAt(0.0f);
    if (initialManifold.pointCount > 0)
        return SweepManifold{0.0f, initialManifold};

    constexpr int32_t coarseSteps = 8;
    constexpr int32_t refinementSteps = 10;
    float fractionLower = 0.0f;
    float fractionUpper = 1.0f;
    Manifold hitManifold{};

    for (int32_t stepIndex = 1; stepIndex <= coarseSteps; ++stepIndex)
    {
        const float testFraction = static_cast<float>(stepIndex) / static_cast<float>(coarseSteps);
        Manifold testManifold = manifoldAt(testFraction);
        if (testManifold.pointCount > 0)
        {
            fractionLower = static_cast<float>(stepIndex - 1) / static_cast<float>(coarseSteps);
            fractionUpper = testFraction;
            hitManifold = testManifold;
            break;
        }

        if (stepIndex == coarseSteps)
            return std::nullopt;
    }

    for (int32_t iteration = 0; iteration < refinementSteps; ++iteration)
    {
        const float midFraction = 0.5f * (fractionLower + fractionUpper);
        Manifold midManifold = manifoldAt(midFraction);
        if (midManifold.pointCount > 0)
        {
            fractionUpper = midFraction;
            hitManifold = midManifold;
        }
        else
        {
            fractionLower = midFraction;
        }

        // Stop if the precision is sufficient
        if (fractionUpper - fractionLower < 1e-6f)
            break;
    }

    return SweepManifold{fractionUpper, hitManifold};
}

} // namespace c2d
