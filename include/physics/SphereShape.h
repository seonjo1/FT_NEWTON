#ifndef SPHERESHAPE_H
#define SPHERESHAPE_H

#include "physics/Shape.h"

namespace ale
{
class SphereShape : public Shape
{
  public:
	SphereShape();
	virtual ~SphereShape() = default;
	SphereShape *clone() const;
	int32_t GetChildCount() const;
	void ComputeAABB(AABB *aabb, const Transform &xf) const;
	void ComputeMass(MassData *massData, float density) const;
	void SetCenter(const glm::vec3 &center);
	void SetRadius(float radius);

	float radius;
};
} // namespace ale
#endif