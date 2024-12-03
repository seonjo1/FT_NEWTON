#include "physics/BroadPhase.h"

namespace ale
{
BroadPhase::BroadPhase()
{
	moveCount = 0;
	moveCapacity = 16;
	moveBuffer.resize(moveCapacity);
}

int32_t BroadPhase::CreateProxy(const AABB &aabb, void *userData)
{
	int32_t proxyId = tree.CreateProxy(aabb, userData);
	BufferMove(proxyId);
	return proxyId;
}

void BroadPhase::DestroyProxy(int32_t proxyId)
{
}

void BroadPhase::MoveProxy(int32_t proxyId, const AABB &aabb, const glm::vec3 &displacement)
{
}

void BroadPhase::BufferMove(int32_t proxyId)
{
	if (moveCount == moveCapacity)
	{
		moveCapacity *= 2;
		moveBuffer.resize(moveCapacity);
	}
	moveBuffer[moveCount] = proxyId;
	++moveCount;
}

// const AABB &BroadPhase::GetFatAABB(int32_t proxyId) const
// {
// }

// bool BroadPhase::TestOverlap(int32_t proxyIdA, int32_t proxyIdB) const
// {
// }

// void *BroadPhase::GetUserData(int32_t proxyId) const
// {
// }

} // namespace ale