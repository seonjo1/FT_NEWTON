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
	void setShapeFeatures(std::vector<Vertex>& vertices);
	virtual float getLocalRadius() const override;
	virtual const glm::vec3& getLocalHalfSize() const override;

	float radius;
	float localRadius;
};
} // namespace ale
#endif