#ifndef CONTACTMANAGER_H
#define CONTACTMANAGER_H

#include "BroadPhase.h"

namespace ale
{

class ContactManager
{
  public:
	ContactManager();
	void AddPair(int32_t proxyA, int32_t proxyB);

	BroadPhase broadPhase;
};

} // namespace ale

#endif