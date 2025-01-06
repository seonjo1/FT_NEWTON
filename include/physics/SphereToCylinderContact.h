#ifndef SPHERETOCYLINDERCONTACT_H
#define SPHERETOCYLINDERCONTACT_H

#include "physics/Contact.h"

namespace ale
{

class SphereToCylinderContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	SphereToCylinderContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &sphere, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &cylinder, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &sphere, const ConvexInfo &cylinder,
									 std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
									 std::vector<Simplex> &simplexVector) override;
};
} // namespace ale

#endif