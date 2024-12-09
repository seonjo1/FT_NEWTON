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
}
} // namespace ale

#endif