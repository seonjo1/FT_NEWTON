#ifndef SPHERESHAPE_H
#define SPHERESHAPE_H

#include "physics/Shape.h"

struct ConvexInfo;

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
	void setShapeFeatures(std::vector<Vertex>& vertices);
	virtual float getLocalRadius() const override;
	virtual const glm::vec3& getLocalHalfSize() const override;
	virtual ConvexInfo getShapeInfo(const Transform &transform) const override;

	float m_radius;
	float localRadius;
};
} // namespace ale
#endif