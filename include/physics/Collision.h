#ifndef COLLISION_H
#define COLLISION_H

#include "common.h"

namespace ale
{
struct AABB
{
	bool IsValid() const;

	glm::vec3 GetCenter() const
	{
	}

	glm::vec3 GetExtents() const
	{
	}

	float GetSurface() const
	{
		glm::vec3 v = upperBound - lowerBound;

		return (v.x * v.y + v.y * v.z + v.z * v.x) * 2.0f;
	}

	void Combine(const AABB &aabb)
	{
		lowerBound = min(lowerBound, aabb.lowerBound);
		upperBound = max(upperBound, aabb.upperBound);
	}

	void Combine(const AABB &aabb1, const AABB &aabb2)
	{
		lowerBound = min(aabb1.lowerBound, aabb2.lowerBound);
		upperBound = max(aabb1.upperBound, aabb2.upperBound);
	}

	bool Contains(const AABB &aabb) const
	{
		bool result = true;

		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;

		return result;
	}

	glm::vec3 lowerBound;
	glm::vec3 upperBound;
};

// translation, rotation
struct Transform
{
	Transform()
	{
	}
	Transform(const glm::vec3 &p, const glm::quat &r) : position(p), orientation(r)
	{
	}
	void Set(const glm::vec3 &p, float angle)
	{
		position = p;
		// set orientation by angle
	}

	glm::vec3 position;
	glm::quat orientation;
};

struct Sweep
{
	glm::vec3 p;
	glm::quat q;
	float alpha;
};

inline bool testOverlap(const AABB &a, const AABB &b)
{
	glm::vec3 d1, d2;

	d1 = b.lowerBound - a.upperBound;
	d2 = a.lowerBound - b.upperBound;

	if (d1.x > 0 || d1.y > 0 || d1.z > 0)
		return false;
	if (d2.x > 0 || d2.y > 0 || d2.z > 0)
		return false;
	return true;
}
} // namespace ale
#endif