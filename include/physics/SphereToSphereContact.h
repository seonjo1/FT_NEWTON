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
	virtual void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB) override;
};
} // namespace ale

#endif