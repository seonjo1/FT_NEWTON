#include "physics/SphereShape.h"

namespace ale
{
SphereShape::SphereShape()
{
	m_type = Type::SPHERE;
}

SphereShape *SphereShape::clone() const
{
	SphereShape *clone = new SphereShape();
	*clone = *this;
	return clone;
}

int32_t SphereShape::getChildCount() const
{
	return 1;
}

void SphereShape::computeAABB(AABB *aabb, const Transform &xf) const
{
	// get min, max vertex
	glm::vec3 upper = xf.position + glm::vec3(m_radius);
	glm::vec3 lower = xf.position - glm::vec3(m_radius);

	// std::cout << "upper: " << upper.x << ", " << upper.y << ", " << upper.z << '\n';
	// std::cout << "lower: " << lower.x << ", " << lower.y << ", " << lower.z << '\n';

	aabb->upperBound = upper + glm::vec3(0.1f);
	aabb->lowerBound = lower - glm::vec3(0.1f);
}
void SphereShape::computeMass(MassData *massData, float density) const
{
}

void SphereShape::setCenter(const glm::vec3 &center)
{
	m_center = center;
}

void SphereShape::setRadius(float radius)
{
	m_radius = radius;
}

} // namespace ale