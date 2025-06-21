#pragma once

#include <algorithm>
#include <random>
#include <ctime>

template <typename Iterator>
void randomShuffle(Iterator begin, Iterator end)
{
	// The random number generator that we want to use (Mersenne Twister)
	std::mt19937 rng(std::time(nullptr));
	
	// Shuffle the numbers
	std::shuffle(begin, end, rng);
}
