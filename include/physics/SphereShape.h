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
	void setShapeFeatures(std::vector<Vertex> &vertices);
	virtual ConvexInfo getShapeInfo(const Transform &transform) const override;

	float m_radius;
};
} // namespace ale
#endif