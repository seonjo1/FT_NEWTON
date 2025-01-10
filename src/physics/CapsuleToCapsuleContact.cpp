#include "physics/CapsuleToCapsuleContact.h"

namespace ale
{

CapsuleToCapsuleContact::CapsuleToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *CapsuleToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new CapsuleToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 CapsuleToCapsuleContact::supportA(const ConvexInfo &capsule, glm::vec3 dir)
{

}

glm::vec3 CapsuleToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
{

}

void CapsuleToCapsuleContact::findCollisionPoints(const ConvexInfo &capsuleA, const ConvexInfo &capsuleB,
												  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
												  std::vector<Simplex> &simplexVector)
{

}

} // namespace ale