#ifndef SPHERESHAPE_H
#define SPHERESHAPE_H

#include "Shape.h"

namespace ale
{
class SphereShape : public Shape
{
  public:
	SphereShape();
	SphereShape *clone() const;
	int32_t GetChildCount() const;
	void ComputeAABB(AABB *aabb) const;
	void ComputeMass(MassData *massData, float density) const;
	void SetCenter(const glm::vec3 &center);

	glm::vec3 center;
	float radius;
};
} // namespace ale
#endif