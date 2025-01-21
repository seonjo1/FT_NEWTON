#include "physics/SphereToSphereContact.h"

namespace ale
{

SphereToSphereContact::SphereToSphereContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToSphereContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(SphereToSphereContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<SphereToSphereContact *>(memory)) SphereToSphereContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 SphereToSphereContact::supportA(const ConvexInfo &sphere, glm::vec3 dir)
{
	return sphere.center + dir * sphere.radius;
}

glm::vec3 SphereToSphereContact::supportB(const ConvexInfo &sphere, glm::vec3 dir)
{
	return sphere.center + dir * sphere.radius;
}

void SphereToSphereContact::findCollisionPoints(const ConvexInfo &sphereA, const ConvexInfo &sphereB,
												CollisionInfo &collisionInfo, EpaInfo &epaInfo,
												SimplexArray &simplexArray)
{
	collisionInfo.normal[0] = epaInfo.normal;
	collisionInfo.seperation[0] = epaInfo.distance;
	collisionInfo.pointA[0] = sphereA.center + epaInfo.normal * sphereA.radius;
	collisionInfo.pointB[0] = collisionInfo.pointA[0] - collisionInfo.normal[0] * collisionInfo.seperation[0];
	++collisionInfo.size;
}
} // namespace ale