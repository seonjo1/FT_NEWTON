#ifndef PHYSICSALLOCATOR_H
#define PHYSICSALLOCATOR_H

#include "BlockAllocator.h"
#include "StackAllocator.h"

namespace ale
{

class PhysicsAllocator
{
  public:
	PhysicsAllocator() = default;
	~PhysicsAllocator() = default;

	static BlockAllocator m_blockAllocator;
	static StackAllocator m_stackAllocator;
};

} // namespace ale
#endif