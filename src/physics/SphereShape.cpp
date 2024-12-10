#include "physics/SphereShape.h"

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

void SphereShape::ComputeAABB(AABB *aabb, const Transform &xf) const
{
	// get min, max vertex
	glm::vec3 upper = center + glm::vec3(radius);
	glm::vec3 lower = center - glm::vec3(radius);

	std::cout << "upper: " << upper.x << ", " << upper.y << ", " << upper.z << '\n';
	std::cout << "lower: " << lower.x << ", " << lower.y << ", " << lower.z << '\n';

	aabb->upperBound = upper + glm::vec3(0.1f);
	aabb->lowerBound = lower - glm::vec3(0.1f);
}
void SphereShape::ComputeMass(MassData *massData, float density) const
{
}

void SphereShape::SetCenter(const glm::vec3 &center)
{
	this->center = center;
}

void SphereShape::SetRadius(float radius)
{
	this->radius = radius;
}

void SphereShape::setShapeFeatures(std::vector<Vertex>& vertices)
{
	// welzl 알고리즘 나중에 적용 고려
	localCenter = glm::vec3(0.0f);
	float distance = 0.0f;

	for (const Vertex &vertex : vertices)
	{
		distance = std::max(
			vertex.position.x * vertex.position.x +
			vertex.position.y * vertex.position.y +
			vertex.position.z * vertex.position.z,
			distance
		);
	}

	localRadius = std::sqrt(distance);
}


} // namespace ale