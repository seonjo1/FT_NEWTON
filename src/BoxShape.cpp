#include "BoxShape.h"

namespace ale
{
BoxShape::BoxShape()
{
	type = Type::e_box;
	// Vertex Info
}
BoxShape *BoxShape::clone() const
{
}
int32_t BoxShape::GetChildCount() const
{
	return 1;
}
void BoxShape::ComputeAABB(AABB *aabb) const
{
	// get min, max vertex
	glm::vec3 upper;
	glm::vec3 lower;

	aabb->upperBound = upper;
	aabb->lowerBound = lower;
}
void BoxShape::ComputeMass(MassData *massData, float density) const
{
}

} // namespace ale