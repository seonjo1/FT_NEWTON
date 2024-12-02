#ifndef CONTACTMANAGER_H
#define CONTACTMANAGER_H

#include "BroadPhase.h"

namespace ale
{

class ContactManager
{
  public:
	ContactManager();
	void AddPair(void proxyA, void proxyB);

	BroadPhase broadPhase;
};

} // namespace ale

#endif