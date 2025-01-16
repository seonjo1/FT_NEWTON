#include "physics/SphereToSphereContact.h"

namespace ale
{

SphereToSphereContact::SphereToSphereContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToSphereContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToSphereContact(fixtureA, fixtureB, indexA, indexB);
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
												std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
												std::vector<Simplex> &simplexVector)
{
	CollisionInfo collisionInfo;

	collisionInfo.normal = epaInfo.normal;
	collisionInfo.seperation = epaInfo.distance;
	collisionInfo.pointA = sphereA.center + epaInfo.normal * sphereA.radius;
	collisionInfo.pointB = collisionInfo.pointA - collisionInfo.normal * collisionInfo.seperation;

	collisionInfoVector.push_back(collisionInfo);
}
} // namespace ale