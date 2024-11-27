#ifndef DYNAMICTREE_H
#define DYNAMICTREE_H

namespace ale
{
struct TreeNode
{
	bool IsLeaf() const;
	AABB aabb; // Enlarged AABB
	void* userData;
	int parent;
	int child1;
	int child2;
	int height;
};

class DynamicTree
{
public:
	// Dynamic Tree 생성
	DynamicTree();
	~DynamicTree();
	
	// 주어진 aabb와 userData로 node에 값 초기화, node 삽입
	int CreateProxy(const AABB& aabb, void* userData);
	
	// proxyId에 해당하는 node Destroy
	void DestroyProxy(int proxyId);
	
	// proxyId에 해당하는 node 삭제 후, 적당한 위치로 다시 Insert
	bool MoveProxy(int proxyId, const AABB& aabb1, const Vec3& displacement);
	
	//
	void* GetUserData(int proxyId) const;
	
	//
	const AABB& GetFatAABB(int proxyId) const;
	
	template<typename T>
	void Query(T* callback, const AABB& aabb) const;
	
private:
	int AllocateNode();
	void FreeNode(int nodeId);
	
	// 가장 최적의 위치를 찾아 node 삽입, Balance
	void InsertLeaf(int node);
	
	// node 삭제
	void RemoveLeaf(int node);
	
	// 트리가 쏠리지 않게 Balance 맞춰줌
	int Balance(int index);
	
	// tree의 height 계산
	int ComputeHeight() const;
	// sub-tree의 height 계산
	int ComputeHeight(int nodeId) const;

	int root;
	std::vector<TreeNode> nodes;
	int nodeCapacity;

};
}

#endif