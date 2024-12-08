#ifndef BROADPHASE_H
#define BROADPHASE_H

#include "DynamicTree.h"
#include "common.h"
#include <utility>

namespace ale
{
class DynamicTree;

class BroadPhase
{
  public:
	enum
	{
		NULL_PROXY = -1
	};

	BroadPhase();

	// AABB에 해당하는 proxy 생성 - DynamicTree의 nodeId를 반환한다
	int32_t CreateProxy(const AABB &aabb, void *userData);

	// proxyId에 해당하는 node Destroy
	void DestroyProxy(int32_t proxyId);

	void MoveProxy(int32_t proxyId, const AABB &aabb, const glm::vec3 &displacement);

	void BufferMove(int32_t proxyId);

	// proxyId에 해당하는 FatAABB 반환
	// const AABB &GetFatAABB(int32_t proxyId) const;

	// proxyId pair끼리 겹치는지 확인
	// bool TestOverlap(int32_t proxyIdA, int32_t proxyIdB) const;

	// proxyId에 해당하는 data get
	// void *GetUserData(int32_t proxyId) const;

	// moved proxy buffer를 순회하며, 가능성 있는 충돌 쌍 검색
	// callback을 사용해 ContactManager의 AddPair 호출
	template <typename T> void UpdatePairs(T *callback);

	// 추후 필요에 따라 수정
	template <typename T> void Query(T *callback, const AABB &aabb) const;

  private:
	friend class DynamicTree;
	bool queryCallback(int32_t proxyId);

	// Dynamic tree
	DynamicTree tree;
	// proxyA, proxyB pair set
	std::set<std::pair<int32_t, int32_t>> proxySet;
	// moved proxy buffer
	std::vector<int32_t> moveBuffer;
	int32_t moveCapacity;
	int32_t moveCount;
	int32_t queryProxyId;
};

template <typename T> void BroadPhase::UpdatePairs(T *callback)
{
	// std::cout << "BroadPhase::UpdatePairs\n";
	// std::cout << "movecount: " << moveCount << '\n';
	for (int32_t i = 0; i < moveCount; ++i)
	{
		queryProxyId = moveBuffer[i];
		std::cout << "queryProxyId: " << queryProxyId << '\n';
		if (queryProxyId == NULL_PROXY)
		{
			continue;
		}

		const AABB &fatAABB = tree.GetFatAABB(queryProxyId);

		tree.Query(this, fatAABB);
	}

	moveCount = 0;
	moveBuffer.clear();
	for (auto &it = proxySet.begin(); it != proxySet.end();)
	{
		auto primaryPair = it;
		std::cout << "proxyIdA: " << primaryPair->first << " proxyIdB: " << primaryPair->second << '\n';
		void *userDataA = tree.GetUserData(primaryPair->first);
		void *userDataB = tree.GetUserData(primaryPair->second);

		callback->AddPair(userDataA, userDataB);
		++it;
		while (it != proxySet.end())
		{
			auto pair = it;

			if (pair->first != primaryPair->first || pair->second != primaryPair->second)
			{
				break;
			}
			++it;
		}
	}
}
} // namespace ale
#endif