#include "physics/PhysicsAllocator.h"

namespace ale
{

// static 멤버 변수 정의
BlockAllocator PhysicsAllocator::m_blockAllocator;
StackAllocator PhysicsAllocator::m_stackAllocator;

}