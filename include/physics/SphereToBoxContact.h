#ifndef SPHERETOBOXCONTACT_H
#define SPHERETOBOXCONTACT_H

#include "physics/Contact.h"

namespace ale
{
class SphereToBoxContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	SphereToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &sphere, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &box, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &sphere, const ConvexInfo &box,
									 std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
									 std::vector<Simplex> &simplexVector) override;
};
} // namespace ale

#endif