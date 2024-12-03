#include "physics/BoxShape.h"

namespace ale
{
BoxShape::BoxShape()
{
	type = Type::e_box;
}
BoxShape *BoxShape::clone() const
{
	BoxShape *clone = new BoxShape();
	*clone = *this;
	return clone;
}
int32_t BoxShape::GetChildCount() const
{
	return 1;
}
void BoxShape::ComputeAABB(AABB *aabb) const
{
	// get min, max vertex
	glm::vec3 upper = *std::prev(vertices.end());
	glm::vec3 lower = *vertices.begin();

	aabb->upperBound = upper;
	aabb->lowerBound = lower;
}
void BoxShape::ComputeMass(MassData *massData, float density) const
{
}

void BoxShape::SetVertices(const std::vector<Vertex> &vertices)
{
	for (const Vertex &vertex : vertices)
	{
		glm::vec3 v = vertex.position;
		this->vertices.insert(v);
	}
}

} // namespace ale