#ifndef CONTACTMANAGER_H
#define CONTACTMANAGER_H

#include "BroadPhase.h"

namespace ale
{

class ContactManager
{
  public:
	ContactManager();
	void AddPair(void *proxyUserDataA, void *proxyUserDataB);
	void findNewContacts();

	BroadPhase broadPhase;
};

} // namespace ale

#endif