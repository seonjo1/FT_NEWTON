#ifndef BOXSHAPE_H
#define BOXSHAPE_H

#include "Shape.h"

namespace ale
{
class BoxShape : public Shape
{
  public:
	BoxShape();
	BoxShape *clone() const;
	int32_t GetChildCount() const;
	void ComputeAABB(AABB *aabb) const;
	void ComputeMass(MassData *massData, float density) const;
	void SetVertices(const std::vector<Vertex> &v);

	// Vertex Info needed
	std::set<glm::vec3> vertices;
};
} // namespace ale

#endif