#include "physics/DynamicTree.h"

namespace ale
{
DynamicTree::DynamicTree()
{
	root = nullNode;
	nodeCapacity = 16;
	nodes.resize(nodeCapacity);
	nodeCount = 0;

	for (int32_t i = 0; i < nodeCapacity - 1; ++i)
	{
		nodes[i].next = i + 1;
		nodes[i].height = -1;
	}
	nodes[nodeCapacity - 1].next = nullNode;
	nodes[nodeCapacity - 1].height = -1;
	freeNode = 0;
}

DynamicTree::~DynamicTree()
{
	nodes.clear();
}

int DynamicTree::AllocateNode()
{
	if (freeNode == nodeCapacity)
	{
		nodeCapacity *= 2;
		nodes.resize(nodeCapacity);

		for (int32_t i = nodeCount; i < nodeCapacity - 1; ++i)
		{
			nodes[i].next = i + 1;
			nodes[i].height = -1;
		}
		nodes[nodeCapacity - 1].next = nullNode;
		nodes[nodeCapacity - 1].height = -1;
		freeNode = nodeCount;
	}

	int32_t nodeId = freeNode;
	freeNode = nodes[nodeId].next;
	nodes[nodeId].parent = nullNode;
	nodes[nodeId].child1 = nullNode;
	nodes[nodeId].child2 = nullNode;
	nodes[nodeId].height = 0;
	nodes[nodeId].userData = nullptr;
	++nodeCount;
	return nodeId;
}

void DynamicTree::FreeNode(int32_t nodeId)
{
	nodes[nodeId].next = freeNode;
	nodes[nodeId].height = -1;
	freeNode = nodeId;
	--nodeCount;
}

int32_t DynamicTree::CreateProxy(const AABB &aabb, void *userData)
{
	std::cout << "DynamicTree::CreateProxy\n";
	int32_t proxyId = AllocateNode();
	std::cout << "proxyId: " << proxyId << '\n';

	glm::vec3 r(0.1, 0.1, 0.1);
	nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
	nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
	nodes[proxyId].userData = userData;
	nodes[proxyId].height = 0;

	// insert leaf
	InsertLeaf(proxyId);

	std::cout << "DynamicTree::CreateProxy end\n";

	return proxyId;
}

void DynamicTree::DestroyProxy(int32_t proxyId)
{
	// remove leaf
	FreeNode(proxyId);
}

bool DynamicTree::MoveProxy(int32_t proxyId, const AABB &aabb, const glm::vec3 &displacement)
{
	if (nodes[proxyId].aabb.contains(aabb))
	{
		return false;
	}

	RemoveLeaf(proxyId);

	AABB b = aabb;
	glm::vec3 r(0.1f);
	b.lowerBound = b.lowerBound - r;
	b.upperBound = b.upperBound + r;

	glm::vec3 d = 2.0f * displacement;

	if (d.x < 0.0f)
	{
		b.lowerBound.x += d.x;
	}
	else
	{
		b.upperBound.x += d.x;
	}

	if (d.y < 0.0f)
	{
		b.lowerBound.y += d.y;
	}
	else
	{
		b.upperBound.y += d.y;
	}

	if (d.z < 0.0f)
	{
		b.lowerBound.z += d.z;
	}
	else
	{
		b.upperBound.z += d.z;
	}

	nodes[proxyId].aabb = b;

	InsertLeaf(proxyId);
	return true;
}

void *DynamicTree::GetUserData(int32_t proxyId) const
{
	// check proxyId range
	return nodes[proxyId].userData;
}

const AABB &DynamicTree::GetFatAABB(int32_t proxyId) const
{
	return nodes[proxyId].aabb;
}

void DynamicTree::InsertLeaf(int32_t leaf)
{
	if (root == nullNode)
	{
		root = leaf;
		nodes[root].parent = nullNode;
		return;
	}

	AABB leafAABB = nodes[leaf].aabb;
	int32_t index = root;

	while (nodes[index].IsLeaf() == false)
	{
		int32_t child1 = nodes[index].child1;
		int32_t child2 = nodes[index].child2;

		float area = nodes[index].aabb.getSurface();
		AABB combinedAABB;
		combinedAABB.combine(nodes[index].aabb, leafAABB);
		float combinedArea = combinedAABB.getSurface();

		float cost = 2.0f * combinedArea;
		float inheritedCost = 2.0f * (combinedArea - area);

		float cost1;
		if (nodes[child1].IsLeaf())
		{
			cost1 = GetInsertionCostForLeaf(leafAABB, child1, inheritedCost);
		}
		else
		{
			cost1 = GetInsertionCost(leafAABB, child1, inheritedCost);
		}

		float cost2;
		if (nodes[child2].IsLeaf())
		{
			cost2 = GetInsertionCostForLeaf(leafAABB, child2, inheritedCost);
		}
		else
		{
			cost2 = GetInsertionCost(leafAABB, child2, inheritedCost);
		}

		if (cost < cost1 && cost < cost2)
			break;

		if (cost1 < cost2)
		{
			index = child1;
		}
		else
		{
			index = child2;
		}
	}

	int32_t sibling = index;

	int32_t oldParent = nodes[sibling].parent;
	int32_t newParent = AllocateNode();

	nodes[newParent].parent = oldParent;
	nodes[newParent].userData = nullptr;
	nodes[newParent].aabb.combine(leafAABB, nodes[sibling].aabb);
	nodes[newParent].height = nodes[sibling].height + 1;

	if (oldParent != nullNode)
	{
		if (nodes[oldParent].child1 == sibling)
		{
			nodes[oldParent].child1 = newParent;
		}
		else
		{
			nodes[oldParent].child2 = newParent;
		}

		nodes[newParent].child1 = sibling;
		nodes[newParent].child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
	}
	else
	{
		nodes[newParent].child1 = sibling;
		nodes[newParent].child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
		root = newParent;
	}

	index = nodes[leaf].parent;
	while (index != nullNode)
	{
		index = Balance(index);

		int32_t child1 = nodes[index].child1;
		int32_t child2 = nodes[index].child2;

		assert(child1 != nullNode);
		assert(child2 != nullNode);

		nodes[index].height = std::max(nodes[child1].height, nodes[child2].height) + 1;
		nodes[index].aabb.combine(nodes[child1].aabb, nodes[child2].aabb);

		index = nodes[index].parent;
	}
	// std::cout << "print tree\n";
	// printDynamicTree(root);
}

void DynamicTree::RemoveLeaf(int32_t leaf)
{
	if (leaf == root)
	{
		root = nullNode;
		return;
	}

	int32_t parent = nodes[leaf].parent;
	int32_t grandParent = nodes[parent].parent;
	int32_t sibling;

	if (nodes[parent].child1 == leaf)
	{
		sibling = nodes[parent].child2;
	}
	else
	{
		sibling = nodes[parent].child1;
	}

	if (grandParent != nullNode)
	{
		if (nodes[grandParent].child1 == parent)
		{
			nodes[grandParent].child1 = sibling;
		}
		else
		{
			nodes[grandParent].child2 = sibling;
		}
		nodes[sibling].parent = grandParent;
		FreeNode(parent);

		int32_t index = grandParent;
		while (index != nullNode)
		{
			index = Balance(index);

			int32_t child1 = nodes[index].child1;
			int32_t child2 = nodes[index].child2;

			nodes[index].height = std::max(nodes[child1].height, nodes[child2].height) + 1;
			nodes[index].aabb.combine(nodes[child1].aabb, nodes[child2].aabb);

			index = nodes[index].parent;
		}
	}
	else
	{
		root = sibling;
		nodes[sibling].parent = nullNode;
		FreeNode(parent);
	}
	// std::cout << "print tree\n";
	// printDynamicTree(root);
}

float DynamicTree::GetInsertionCostForLeaf(const AABB &leafAABB, int32_t child, float inheritedCost)
{
	AABB aabb;
	aabb.combine(leafAABB, nodes[child].aabb);
	return aabb.getSurface() + inheritedCost;
}

float DynamicTree::GetInsertionCost(const AABB &leafAABB, int32_t child, float inheritedCost)
{
	AABB aabb;
	aabb.combine(leafAABB, nodes[child].aabb);
	float oldArea = nodes[child].aabb.getSurface();
	float newArea = aabb.getSurface();
	return (newArea - oldArea) + inheritedCost;
}

void DynamicTree::printDynamicTree(int32_t nodeId)
{
	if (nodeId == nullNode)
	{
		return;
	}
	std::cout << nodeId << "\n";
	printDynamicTree(nodes[nodeId].child1);
	printDynamicTree(nodes[nodeId].child2);
}

int32_t DynamicTree::Balance(int32_t iA)
{
	TreeNode *A = &nodes[iA];

	if (A->IsLeaf() || A->height < 2)
	{
		return iA;
	}

	int32_t iB = A->child1;
	int32_t iC = A->child2;

	TreeNode *B = &nodes[iB];
	TreeNode *C = &nodes[iC];

	int32_t balance = C->height - B->height;

	if (balance > 1)
	{
		int32_t iF = nodes[iC].child1;
		int32_t iG = nodes[iC].child2;

		TreeNode *F = &nodes[iF];
		TreeNode *G = &nodes[iG];

		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		if (C->parent != nullNode)
		{
			if (nodes[C->parent].child1 == iA)
			{
				nodes[C->parent].child1 = iC;
			}
			else
			{
				nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			root = iC;
		}

		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			nodes[iG].parent = iA;
			A->aabb.combine(B->aabb, G->aabb);
			C->aabb.combine(A->aabb, F->aabb);

			A->height = std::max(B->height, G->height) + 1;
			C->height = std::max(A->height, F->height) + 1;
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb.combine(B->aabb, F->aabb);
			C->aabb.combine(A->aabb, G->aabb);

			A->height = std::max(B->height, F->height) + 1;
			C->height = std::max(A->height, G->height) + 1;
		}
		return iC;
	}

	if (balance < -1)
	{
		int32_t iD = nodes[iB].child1;
		int32_t iE = nodes[iB].child2;

		TreeNode *D = &nodes[iD];
		TreeNode *E = &nodes[iE];

		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		if (B->parent != nullNode)
		{
			if (nodes[B->parent].child1 == iA)
			{
				nodes[B->parent].child1 = iB;
			}
			else
			{
				nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			root = iB;
		}

		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb.combine(C->aabb, E->aabb);
			B->aabb.combine(A->aabb, D->aabb);

			A->height = std::max(C->height, E->height) + 1;
			B->height = std::max(A->height, D->height) + 1;
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb.combine(C->aabb, D->aabb);
			B->aabb.combine(A->aabb, E->aabb);

			A->height = std::max(C->height, D->height) + 1;
			B->height = std::max(A->height, E->height) + 1;
		}
		return iB;
	}
	return iA;
}

} // namespace ale