#ifndef BOXSHAPE_H
#define BOXSHAPE_H

#include "physics/Shape.h"

namespace ale
{
class BoxShape : public Shape
{
  public:
	BoxShape();
	virtual ~BoxShape() = default;
	BoxShape *clone() const;
	int32_t getChildCount() const;
	void computeAABB(AABB *aabb, const Transform &xf) const;
	void setVertices(const std::vector<Vertex> &v);
	virtual ConvexInfo getShapeInfo(const Transform &transform) const override;

	// Vertex Info needed
	std::set<glm::vec3, Vec3Comparator> m_vertices;
	glm::vec3 m_halfSize;
};
} // namespace ale

#endif