#pragma once

#include <cassert>
#include <format>
#include <string>

namespace
{
    float inverseSqrt(float number)
    {
        const float threehalfs = 1.5F; 

        float x2 = number * 0.5F;
        float y = number;
        
        // evil floating point bit level hacking
        long i = * ( long * ) &y;
        
        // value is pre-assumed
        i = 0x5f3759df - ( i >> 1 );
        y = * ( float * ) &i;
        
        // 1st iteration
        y = y * ( threehalfs - ( x2 * y * y ) );
        
        // 2nd iteration, this can be removed
        y = y * ( threehalfs - ( x2 * y * y ) );

        return y;
    }
}

namespace c2d
{

class Vec2
{
public:
	float x;
	float y;

	constexpr Vec2() : Vec2(0, 0) {}
	constexpr Vec2(float x, float y) : x(x), y(y) {}
	
	constexpr Vec2(const Vec2& other) = default;
	constexpr Vec2& operator=(const Vec2& other) = default;
	constexpr Vec2(Vec2&& other) noexcept = default;
	constexpr Vec2& operator=(Vec2&& other) noexcept = default;

	std::string toString() const;

	constexpr Vec2 interpolate(Vec2 other, float percentage) const;
	constexpr Vec2 normalize() const;

	constexpr float getBigger() const;

	constexpr Vec2& operator+=(Vec2 other);
	constexpr Vec2& operator-=(Vec2 other);
	constexpr Vec2& operator*=(Vec2 other);
	constexpr Vec2& operator/=(Vec2 other);
	constexpr Vec2& operator+=(float value);
	constexpr Vec2& operator-=(float value);
	constexpr Vec2& operator*=(float value);
	constexpr Vec2& operator/=(float value);

	constexpr Vec2 operator+(Vec2 other) const;
	constexpr Vec2 operator-(Vec2 other) const;
	constexpr Vec2 operator*(Vec2 other) const;
	constexpr Vec2 operator/(Vec2 other) const;
	constexpr Vec2 operator+(float value) const;
	constexpr Vec2 operator-(float value) const;
	constexpr Vec2 operator*(float value) const;
	constexpr Vec2 operator/(float value) const;

	constexpr bool operator==(Vec2 other) const;
};

inline std::string Vec2::toString() const
{
	return std::format("Vec2(x={}, y={})", x, y);
}

constexpr Vec2 Vec2::interpolate(Vec2 other, float percentage) const
{
    return Vec2(x + (other.x - x) * percentage, y + (other.y - y) * percentage);
}

constexpr Vec2 Vec2::normalize() const
{
	const float length = x * x + y * y;
	const float multiplier = inverseSqrt(length);
	return Vec2(x * multiplier, y * multiplier);
}

constexpr float Vec2::getBigger() const
{
	return x > y ? x : y;
}

// Arithmetic compound assignment
constexpr Vec2& Vec2::operator+=(Vec2 other)
{
	x += other.x;
	y += other.y;
	return *this;
}

constexpr Vec2& Vec2::operator-=(Vec2 other)
{
	x -= other.x;
	y -= other.y;
	return *this;
}

constexpr Vec2& Vec2::operator*=(Vec2 other)
{
	x *= other.x;
	y *= other.y;
	return *this;
}

constexpr Vec2& Vec2::operator/=(Vec2 other)
{
	x /= other.x;
	y /= other.y;
	return *this;
}

// Scalar compound assignment
constexpr Vec2& Vec2::operator+=(float value)
{
	x += value;
	y += value;
	return *this;
}

constexpr Vec2& Vec2::operator-=(float value)
{
	x -= value;
	y -= value;
	return *this;
}

constexpr Vec2& Vec2::operator*=(float value)
{
	x *= value;
	y *= value;
	return *this;
}

constexpr Vec2& Vec2::operator/=(float value)
{
	x /= value;
	y /= value;
	return *this;
}

constexpr Vec2 Vec2::operator+(Vec2 other) const
{
	Vec2 result = *this;
	result += other;
	return result;
}

constexpr Vec2 Vec2::operator-(Vec2 other) const
{
	Vec2 result = *this;
	result -= other;
	return result;
}

constexpr Vec2 Vec2::operator*(Vec2 other) const
{
	Vec2 result = *this;
	result *= other;
	return result;
}

constexpr Vec2 Vec2::operator/(Vec2 other) const
{
	Vec2 result = *this;
	result /= other;
	return result;
}

constexpr Vec2 Vec2::operator+(float value) const
{
	Vec2 result = *this;
	result += value;
	return result;
}

constexpr Vec2 Vec2::operator-(float value) const
{
	Vec2 result = *this;
	result -= value;
	return result;
}

constexpr Vec2 Vec2::operator*(float value) const
{
	Vec2 result = *this;
	result *= value;
	return result;
}

constexpr Vec2 Vec2::operator/(float value) const
{
	Vec2 result = *this;
	result /= value;
	return result;
}

constexpr bool Vec2::operator==(Vec2 other) const
{
	return (x == other.x) && (y == other.y);
}

static_assert(std::is_trivially_copyable_v<Vec2>, "Vec2 must be trivially copyable");

}
