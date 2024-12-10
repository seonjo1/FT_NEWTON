#include "physics/SphereToSphereContact.h"

namespace ale
{

SphereToSphereContact::SphereToSphereContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToSphereContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToSphereContact(fixtureA, fixtureB, indexA, indexB);
}

void SphereToSphereContact::evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB)
{
	
}

} // namespace ale