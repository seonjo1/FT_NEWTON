#ifndef BROADPHASE_H
#define BROADPHASE_H

class DynamicTree;

class BroadPhase
{
public:

	// AABB에 해당하는 proxy 생성 - DynamicTree의 nodeId를 반환한다
	int CreateProxy(const AABB& aabb, void* userData);
	
	// proxyId에 해당하는 node Destroy
	void DestroyProxy(int proxyId);
	
	void MoveProxy(int proxyId, const AABB& aabb, const Vec3& displacement);
	
	// proxyId에 해당하는 FatAABB 반환
	const AABB& GetFatAABB(int proxyId) const;
	
	// proxyId pair끼리 겹치는지 확인
	bool TestOverlap(int proxyIdA, int proxyIdB) const;

	// proxyId에 해당하는 data get
	void* GetUserData(int proxyId) const;
	
	// moved proxy buffer를 순회하며, 가능성 있는 충돌 쌍 검색
	// callback을 사용해 ContactManager의 AddPair 호출
	template <typename T>
	void UpdatePairs(T* callback);
	
	// 추후 필요에 따라 수정
	template <typename T>
	void Query(T* callback, const AABB& aabb) const;

private:
	// Dynamic tree
	DynamicTree tree;
	// proxyA, proxyB pair set
	std::set<pair<int, int> > proxySet;
	// moved proxy buffer
	std::vector<int> moveBuffer;

};

#endif