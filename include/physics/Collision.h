#ifndef COLLISION_H
#define COLLISION_H

#include <glm/glm.hpp>

struct AABB
{
	bool IsValid() const;
	glm::vec3 GetCneter() const;
	glm::vec3 GetExtents() const;
	float GetPerimieter() const;
	void Combine(const AABB& aabb);
	void Combine(const AABB& aabb1, const AABB& aabb2);
	bool Contains(const AABB& aabb) const;
	glm::vec3 lowerBound;
	glm::vec3 upperBound;
};

#endif