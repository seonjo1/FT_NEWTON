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
	glm::vec3 upper = *std::prev(vertices.end()) + center;
	glm::vec3 lower = *vertices.begin() + center;

	std::cout << "upper: " << upper.x << ", " << upper.y << ", " << upper.z << '\n';
	std::cout << "lower: " << lower.x << ", " << lower.y << ", " << lower.z << '\n';

	aabb->upperBound = upper + glm::vec3(0.1f);
	aabb->lowerBound = lower - glm::vec3(0.1f);
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