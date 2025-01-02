#ifndef BOXSHAPE_H
#define BOXSHAPE_H

#include "physics/Shape.h"

struct Vec3Comparator
{
	bool operator()(const glm::vec3 &lhs, const glm::vec3 &rhs) const
	{
		if (lhs.x != rhs.x)
			return lhs.x < rhs.x;
		if (lhs.y != rhs.y)
			return lhs.y < rhs.y;
		return lhs.z < rhs.z;
	}
};

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