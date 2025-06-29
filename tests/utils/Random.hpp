#pragma once

#include <algorithm>
#include <random>
#include <ctime>

#include "colli2de/Vec2.hpp"
#include "colli2de/Shapes.hpp"
#include "geometry/AABB.hpp"
#include "geometry/Ray.hpp"

using namespace c2d;

class DeterministicRNG {
    uint32_t state;
public:
    DeterministicRNG(uint32_t seed = 12345) : state(seed) {}

    // Returns a float in [0, 1)
    float next()
    {
        // LCG parameters (from Numerical Recipes or Box2D)
        state = 1664525 * state + 1013904223;
        return (state & 0x00FFFFFF) / float(0x01000000); // 24-bit precision
    }

    // Returns a float in [min, max)
    float nextFloat(float min, float max)
    {
        return min + (max - min) * next();
    }
};

template <typename Iterator>
void randomShuffle(Iterator begin, Iterator end)
{
	// The random number generator that we want to use (Mersenne Twister)
	std::mt19937 rng(std::time(nullptr));
	
	// Shuffle the numbers
	std::shuffle(begin, end, rng);
}

inline std::vector<AABB> generateRandomAABBs(size_t count, float regionMin, float regionMax, float size, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<AABB> aabbs;
    aabbs.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        float x = rng.nextFloat(regionMin, regionMax - size);
        float y = rng.nextFloat(regionMin, regionMax - size);
        aabbs.push_back(AABB{Vec2{ x, y }, Vec2{x + size, y + size }});
    }
    return aabbs;
}

inline std::vector<Circle> generateRandomCircles(size_t count, float regionMin, float regionMax, float radius, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<Circle> circles;
    circles.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        Vec2 center{rng.nextFloat(regionMin, regionMax), rng.nextFloat(regionMin, regionMax)};
        circles.push_back(Circle{center, radius});
    }

    return circles;
}

inline std::vector<Segment> generateRandomSegments(size_t count, float regionMin, float regionMax, float length, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<Segment> segments;
    segments.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        Vec2 start{rng.nextFloat(regionMin, regionMax), rng.nextFloat(regionMin, regionMax)};
        float angle = rng.nextFloat(0.0f, 2.0f * PI);
        Vec2 end = start + length * Vec2{std::cos(angle), std::sin(angle)};
        segments.push_back(Segment{start, end});
    }

    return segments;
}

inline std::vector<Capsule> generateRandomCapsules(size_t count, float regionMin, float regionMax, float length, float radius, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<Capsule> capsules;
    capsules.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        Vec2 start{rng.nextFloat(regionMin, regionMax), rng.nextFloat(regionMin, regionMax)};
        float angle = rng.nextFloat(0.0f, 2.0f * PI);
        Vec2 end = start + length * Vec2{std::cos(angle), std::sin(angle)};
        capsules.push_back(Capsule{start, end, radius});
    }

    return capsules;
}

inline std::vector<Polygon> generateRandomRectangles(size_t count, float regionMin, float regionMax, float halfSize, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<Polygon> polygons;
    polygons.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        Vec2 center{rng.nextFloat(regionMin, regionMax), rng.nextFloat(regionMin, regionMax)};
        float angle = rng.nextFloat(0.0f, 2.0f * PI);
        polygons.push_back(makeRectangle(center, halfSize, halfSize, angle));
    }

    return polygons;
}

inline std::vector<Ray> generateRandomRays(size_t count, float regionMin, float regionMax, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<Ray> rays;
    rays.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        Vec2 start{rng.nextFloat(regionMin, regionMax), rng.nextFloat(regionMin, regionMax)};
        Vec2 end{rng.nextFloat(regionMin, regionMax), rng.nextFloat(regionMin, regionMax)};
        rays.push_back(Ray{start, end});
    }

    return rays;
}

inline std::vector<InfiniteRay> generateRandomInfiniteRays(size_t count, float regionMin, float regionMax, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<InfiniteRay> rays;
    rays.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        Vec2 start{rng.nextFloat(regionMin, regionMax), rng.nextFloat(regionMin, regionMax)};
        float angle = rng.nextFloat(0.0f, 2.0f * PI);
        Vec2 direction{std::cos(angle), std::sin(angle)};
        rays.push_back(InfiniteRay{start, direction});
    }

    return rays;
}

inline std::vector<Vec2> generateRandomTranslations(size_t count, float minValue, float maxValue, uint32_t seed = 42)
{
    DeterministicRNG rng(seed);
    std::vector<Vec2> translations;
    translations.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        translations.push_back(Vec2{ rng.nextFloat(minValue, maxValue), rng.nextFloat(minValue, maxValue) });
    }

    return translations;
}
