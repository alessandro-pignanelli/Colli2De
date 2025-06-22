#pragma once

#include <algorithm>
#include <random>
#include <ctime>

#include "colli2de/Vec2.hpp"
#include "geometry/AABB.hpp"

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
