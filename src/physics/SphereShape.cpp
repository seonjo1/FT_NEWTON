#include "physics/SphereShape.h"
#include "physics/Contact.h"

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

void SphereShape::setShapeFeatures(std::vector<Vertex> &vertices)
{
	// welzl 알고리즘 나중에 적용 고려
	localCenter = glm::vec3(0.0f);
	float distance = 0.0f;

	for (const Vertex &vertex : vertices)
	{
		distance = std::max(vertex.position.x * vertex.position.x + vertex.position.y * vertex.position.y +
								vertex.position.z * vertex.position.z,
							distance);
	}

	localRadius = std::sqrt(distance);
}

float SphereShape::getLocalRadius() const
{
	return localRadius;
}

const glm::vec3 &SphereShape::getLocalHalfSize() const
{
	return glm::vec3(0.0f);
}

ConvexInfo SphereShape::getShapeInfo(const Transform &transform) const
{
	ConvexInfo sphere;
	sphere.radius = localRadius;
	sphere.center = transform.toMatrix() * glm::vec4(localCenter, 1.0f);
	return sphere;
}

} // namespace ale