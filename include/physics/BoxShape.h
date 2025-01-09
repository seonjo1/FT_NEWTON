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
	void computeMass(MassData *massData, float density) const;
	void setVertices(const std::vector<Vertex> &v);
	virtual float getLocalRadius() const override;
	virtual const glm::vec3& getLocalHalfSize() const override;
	virtual ConvexInfo getShapeInfo(const Transform &transform) const override;

	// Vertex Info needed
	std::set<glm::vec3, Vec3Comparator> m_vertices;
	glm::vec3 halfSize;
};
} // namespace ale

#endif