#include "SphereShape.h"

namespace ale
{
SphereShape::SphereShape()
{
	type = Type::e_sphere;
}

SphereShape *SphereShape::clone() const
{
	SphereShape *clone = new SphereShape();
	*clone = *this;
	return clone;
}

int32_t SphereShape::GetChildCount() const
{
	return 1;
}

void SphereShape::ComputeAABB(AABB *aabb, const Transform &xf, int32_t childIndex) const
{
}
void SphereShape::ComputeMass(MassData *massData, float density) const
{
}

void SphereShape::SetCenter(const glm::vec3 &center)
{
	this->center = center;
}
} // namespace ale