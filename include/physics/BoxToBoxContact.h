#ifndef BOXTOBOXCONTACT_H
#define BOXTOBOXCONTACT_H

#include "Contact.h"

namespace ale
{
class BoxToBoxContact : public Contact
{
public:
	static Contact* create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	BoxToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	virtual void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB) override;
};
}

#endif