#ifndef CYLINDERTOCAPSULECONTACT_H
#define CYLINDERTOCAPSULECONTACT_H

#include "physics/Contact.h"

namespace ale
{

class CylinderToCapsuleContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	CylinderToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &cylinder, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &capsule, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &cylinder, const ConvexInfo &capsule,
									 CollisionInfo &collisionInfo, EpaInfo &epaInfo,
									 SimplexArray &simplexArray) override;
};
} // namespace ale

#endif