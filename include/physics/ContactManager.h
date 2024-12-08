#ifndef CONTACTMANAGER_H
#define CONTACTMANAGER_H

#include "BroadPhase.h"
#include "Contact.h"
#include "Shape.h"

namespace ale
{

class ContactManager
{
  public:
	ContactManager();
	void addPair(void *proxyUserDataA, void *proxyUserDataB);
	void findNewContacts();
	bool isSameContact(ContactLink *link, Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	void collide();

	BroadPhase broadPhase;
	Contact *m_contactList;
	int32_t m_contactCount;
};

} // namespace ale

#endif