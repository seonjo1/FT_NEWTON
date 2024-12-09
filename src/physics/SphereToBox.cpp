#include "physics/SphereToBoxContact.h"

namespace ale
{

SphereToBoxContact::SphereToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToBoxContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToBoxContact(fixtureA, fixtureB, indexA, indexB);
}

} // namespace ale