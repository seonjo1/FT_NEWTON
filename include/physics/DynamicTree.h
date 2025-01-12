#ifndef DYNAMICTREE_H
#define DYNAMICTREE_H

#include "Collision.h"
#include "common.h"
#include <stack>

#define nullNode (-1)

namespace ale
{
struct TreeNode
{
	bool IsLeaf() const
	{
		return child1 == nullNode;
	}
	AABB aabb; // Enlarged AABB
	void *userData;
	union {
		int32_t parent;
		int32_t next;
	};
	int32_t child1;
	int32_t child2;
	int32_t height;
};

class DynamicTree
{
  public:
	// Dynamic Tree 생성
	DynamicTree();
	~DynamicTree();

	// 주어진 aabb와 userData로 node에 값 초기화, node 삽입
	int32_t CreateProxy(const AABB &aabb, void *userData);

	// proxyId에 해당하는 node Destroy
	void DestroyProxy(int32_t proxyId);

	// proxyId에 해당하는 node 삭제 후, 적당한 위치로 다시 Insert
	bool MoveProxy(int32_t proxyId, const AABB &aabb, const glm::vec3 &displacement);

	//
	void *GetUserData(int32_t proxyId) const;

	//
	const AABB &GetFatAABB(int32_t proxyId) const;

	template <typename T> void Query(T *callback, const AABB &aabb) const;

  private:
	int32_t AllocateNode();
	void FreeNode(int32_t nodeId);

	// 가장 최적의 위치를 찾아 node 삽입, Balance
	void InsertLeaf(int32_t leaf);

	// node 삭제
	void RemoveLeaf(int32_t leaf);

	// 트리가 쏠리지 않게 Balance 맞춰줌
	int32_t Balance(int32_t index);

	float GetInsertionCostForLeaf(const AABB &leafAABB, int32_t child, float inheritedCost);

	float GetInsertionCost(const AABB &leafAABB, int32_t child, float inheritedCost);

	void printDynamicTree(int32_t node);

	// tree의 height 계산
	int32_t ComputeHeight() const;
	// sub-tree의 height 계산
	int32_t ComputeHeight(int32_t nodeId) const;

	int32_t root;
	int32_t freeNode;
	std::vector<TreeNode> nodes;
	int32_t nodeCapacity;
	int32_t nodeCount;
};

template <typename T> inline void DynamicTree::Query(T *callback, const AABB &aabb) const
{
	// std::cout << "DynamicTree::Query\n";
	std::stack<int32_t> stack;
	stack.push(root);

	while (!stack.empty())
	{
		int32_t nodeId = stack.top();
		stack.pop();
		// std::cout << "nodeId: " << nodeId << '\n';
		if (nodeId == nullNode)
		{
			continue;
		}

		const TreeNode node = nodes[nodeId];
		if (testOverlap(node.aabb, aabb))
		{
			if (node.IsLeaf())
			{
				bool proceed = callback->queryCallback(nodeId);
				if (proceed == false)
				{
					return;
				}
			}
			else
			{
				// std::cout << "stack push: " << node.child1 << '\n';
				stack.push(node.child1);
				// std::cout << "stack push: " << node.child2 << '\n';
				stack.push(node.child2);
			}
		}
	}
}
} // namespace ale

#endif