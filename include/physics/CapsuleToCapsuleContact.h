#ifndef CAPSULETOCAPSULECONTACT_H
#define CAPSULETOCAPSULECONTACT_H

#include "physics/Contact.h"

namespace ale
{

class CapsuleToCapsuleContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	CapsuleToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &capsule, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &capsule, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &capsuleA, const ConvexInfo &capsuleB,
									 std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
									 std::vector<Simplex> &simplexVector) override;
};
} // namespace ale

#endif