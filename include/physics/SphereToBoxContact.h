#ifndef SPHERETOBOXCONTACT_H
#define SPHERETOBOXCONTACT_H

#include "Contact.h"

namespace ale
{
class SphereToBoxContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	SphereToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	virtual void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB) override;
}
} // namespace ale

#endif