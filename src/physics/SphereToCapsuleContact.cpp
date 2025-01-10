#include "physics/SphereToCapsuleContact.h"

namespace ale
{

SphereToCapsuleContact::SphereToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 SphereToCapsuleContact::supportA(const ConvexInfo &sphere, glm::vec3 dir)
{
	return sphere.center + dir * sphere.radius;
}

glm::vec3 SphereToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
{

}

void SphereToCapsuleContact::findCollisionPoints(const ConvexInfo &sphere, const ConvexInfo &capsule,
												  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
												  std::vector<Simplex> &simplexVector)
{

}

} // namespace ale