#include "physics/SphereToBoxContact.h"

namespace ale
{

SphereToBoxContact::SphereToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToBoxContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(SphereToBoxContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<SphereToBoxContact *>(memory)) SphereToBoxContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 SphereToBoxContact::supportA(const ConvexInfo &sphere, glm::vec3 dir)
{
	return sphere.center + dir * sphere.radius;
}

glm::vec3 SphereToBoxContact::supportB(const ConvexInfo &box, glm::vec3 dir)
{
	float dotAxes[3] = {glm::dot(box.axes[0], dir) > 0 ? 1.0f : -1.0f, glm::dot(box.axes[1], dir) > 0 ? 1.0f : -1.0f,
						glm::dot(box.axes[2], dir) > 0 ? 1.0f : -1.0f};

	glm::vec3 point = box.center;
	for (int i = 0; i < 3; ++i)
	{
		point += box.axes[i] * (dotAxes[i] * box.halfSize[i]);
	}

	return point;
}

void SphereToBoxContact::findCollisionPoints(const ConvexInfo &sphere, const ConvexInfo &box,
											 CollisionInfo &collisionInfo, EpaInfo &epaInfo, SimplexArray &simplexArray)
{
	collisionInfo.normal[0] = epaInfo.normal;
	collisionInfo.seperation[0] = epaInfo.distance;
	collisionInfo.pointA[0] = sphere.center + epaInfo.normal * sphere.radius;
	collisionInfo.pointB[0] = collisionInfo.pointA[0] - collisionInfo.normal[0] * collisionInfo.seperation[0];
	++collisionInfo.size;
}

} // namespace ale