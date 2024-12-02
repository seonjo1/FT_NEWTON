#ifndef SPHERESHAPE_H
#define SPHERESHAPE_H

#include "Shape.h"
#include "Vertex.h"

namespace ale
{
class SphereShape : public Shape
{
  public:
	SphereShape();
	SphereShape *clone() const;
	int32_t GetChildCount() const;
	void ComputeAABB(AABB *aabb) const;
	void ComputeMass(MassData *massData, float density) const;
	void SetVertices(const std::vector<Vertex> &v);

	std::set<Vertex> vertices;
	glm::vec3 center;
	float radius;
};
} // namespace ale
#endif