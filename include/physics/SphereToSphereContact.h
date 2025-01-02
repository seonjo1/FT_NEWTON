#ifndef SPHERETOSPHERECONTACT_H
#define SPHERETOSPHERECONTACT_H

#include "Contact.h"

namespace ale
{
class SphereToSphereContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	SphereToSphereContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &sphere, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &sphere, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &sphereA, const ConvexInfo &sphereB, std::vector<CollisionInfo> &collisionInfoVector,
							 EpaInfo &epaInfo, std::vector<Simplex> &simplexVector) override;
};
} // namespace ale

#endif