#ifndef BOXTOCYLINDERCONTACT_H
#define BOXTOCYLINDERCONTACT_H

#include "physics/Contact.h"

namespace ale
{

class BoxToCylinderContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	BoxToCylinderContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &box, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &cylinder, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &box, const ConvexInfo &cylinder, CollisionInfo &collisionInfo,
									 EpaInfo &epaInfo, SimplexArray &simplexArray) override;
};
} // namespace ale

#endif