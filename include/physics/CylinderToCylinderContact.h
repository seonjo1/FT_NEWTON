#ifndef CYLINDERTOCYLINDERCONTACT_H
#define CYLINDERTOCYLINDERCONTACT_H

#include "physics/Contact.h"

namespace ale
{

class CylinderToCylinderContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	CylinderToCylinderContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &cylinder, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &cylinder, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &cylinderA, const ConvexInfo &cylinderB,
									 CollisionInfo &collisionInfo, EpaInfo &epaInfo,
									 SimplexArray &simplexArray) override;
};
} // namespace ale

#endif