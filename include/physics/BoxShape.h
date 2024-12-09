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
	int32_t GetChildCount() const;
	void ComputeAABB(AABB *aabb, const Transform &xf) const;
	void ComputeMass(MassData *massData, float density) const;
	void SetVertices(const std::vector<Vertex> &v);

	// Vertex Info needed
	std::set<glm::vec3, Vec3Comparator> vertices;
};
} // namespace ale

#endif