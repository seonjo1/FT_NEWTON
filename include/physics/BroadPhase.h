#ifndef BROADPHASE_H
#define BROADPHASE_H

#include "common.h"
#include "physics/DynamicTree.h"
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
	int32_t createProxy(const AABB &aabb, void *userData);

	// proxyId에 해당하는 node Destroy
	void destroyProxy(int32_t proxyId);

	void moveProxy(int32_t proxyId, const AABB &aabb, const glm::vec3 &displacement);

	void bufferMove(int32_t proxyId);

	// proxyId에 해당하는 FatAABB 반환
	// const AABB &GetFatAABB(int32_t proxyId) const;

	// proxyId pair끼리 겹치는지 확인
	// bool TestOverlap(int32_t proxyIdA, int32_t proxyIdB) const;

	// proxyId에 해당하는 data get
	// void *GetUserData(int32_t proxyId) const;

	// moved proxy buffer를 순회하며, 가능성 있는 충돌 쌍 검색
	// callback을 사용해 ContactManager의 AddPair 호출
	template <typename T> void updatePairs(T *callback);

	// 추후 필요에 따라 수정
	template <typename T> void query(T *callback, const AABB &aabb) const;

  private:
	friend class DynamicTree;
	bool queryCallback(int32_t proxyId);

	DynamicTree m_tree;
	std::set<std::pair<int32_t, int32_t>> m_proxySet;
	std::vector<int32_t> m_moveBuffer;

	int32_t m_moveCapacity;
	int32_t m_moveCount;
	int32_t m_queryProxyId;
};

template <typename T> void BroadPhase::updatePairs(T *callback)
{
	for (int32_t i = 0; i < m_moveCount; ++i)
	{
		m_queryProxyId = m_moveBuffer[i];
		if (m_queryProxyId == NULL_PROXY)
		{
			continue;
		}

		const AABB &fatAABB = m_tree.GetFatAABB(m_queryProxyId);

		m_tree.Query(this, fatAABB);
	}

	m_moveCount = 0;
	for (auto &it = m_proxySet.begin(); it != m_proxySet.end();)
	{
		auto primaryPair = it;
		// std::cout << "proxyIdA: " << primaryPair->first << " proxyIdB: " << primaryPair->second << '\n';
		void *userDataA = m_tree.GetUserData(primaryPair->first);
		void *userDataB = m_tree.GetUserData(primaryPair->second);

		callback->addPair(userDataA, userDataB);
		++it;
		while (it != m_proxySet.end())
		{
			auto pair = it;

			if (pair->first != primaryPair->first || pair->second != primaryPair->second)
			{
				break;
			}
			++it;
		}
	}

	// void *userDataA = m_tree.GetUserData(0);
	// void *userDataB = m_tree.GetUserData(1);
	// callback->addPair(userDataA, userDataB);

	// void *userDataC = m_tree.GetUserData(0);
	// void *userDataD = m_tree.GetUserData(3);
	// callback->addPair(userDataC, userDataD);

	// void *userDataE = m_tree.GetUserData(0);
	// void *userDataF = m_tree.GetUserData(5);
	// callback->addPair(userDataE, userDataF);

	// void *userDataG = m_tree.GetUserData(1);
	// void *userDataH = m_tree.GetUserData(3);
	// callback->addPair(userDataG, userDataH);

	// void *userData6 = m_tree.GetUserData(1);
	// void *userData7 = m_tree.GetUserData(5);
	// callback->addPair(userData6, userData7);

	// void *userData8 = m_tree.GetUserData(3);
	// void *userData9 = m_tree.GetUserData(5);
	// callback->addPair(userData8, userData9);
}
} // namespace ale
#endif