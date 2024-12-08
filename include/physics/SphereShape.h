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
	int32_t getChildCount() const;
	void computeAABB(AABB *aabb, const Transform &xf) const;
	void computeMass(MassData *massData, float density) const;
	void setCenter(const glm::vec3 &center);
	void setRadius(float radius);

	float m_radius;
};
} // namespace ale
#endif