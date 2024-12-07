#include "physics/ContactManager.h"
#include "physics/Fixture.h"

namespace ale
{
ContactManager::ContactManager()
{
}

void ContactManager::AddPair(void *proxyUserDataA, void *proxyUserDataB)
{
	FixtureProxy *proxyA = static_cast<FixtureProxy *>(proxyUserDataA);
	FixtureProxy *proxyB = static_cast<FixtureProxy *>(proxyUserDataB);

	Fixture *fixtureA = proxyA->fixture;
	Fixture *fixtureB = proxyB->fixture;

	int32_t indexA = proxyA->childIndex;
	int32_t indexB = proxyB->childIndex;

	Rigidbody *bodyA = fixtureA->getBody();
	Rigidbody *bodyB = fixtureB->getBody();

	// std::cout << "A - lowerbound\n";
	// std::cout << proxyA->aabb.lowerBound.x << ", " << proxyA->aabb.lowerBound.y << ", " << proxyA->aabb.lowerBound.z
	// 		  << "\n";
	// std::cout << "A - upperbound\n";
	// std::cout << proxyA->aabb.upperBound.x << ", " << proxyA->aabb.upperBound.y << ", " << proxyA->aabb.upperBound.z
	// 		  << "\n";

	// std::cout << "B - lowerbound\n";
	// std::cout << proxyB->aabb.lowerBound.x << ", " << proxyB->aabb.lowerBound.y << ", " << proxyB->aabb.lowerBound.z
	// 		  << "\n";
	// std::cout << "B - upperbound\n";
	// std::cout << proxyB->aabb.upperBound.x << ", " << proxyB->aabb.upperBound.y << ", " << proxyB->aabb.upperBound.z
	// 		  << "\n";

	if (bodyA == bodyB)
	{
		return;
	}
}

void ContactManager::findNewContacts()
{
	// std::cout << "ContactManager::findNewContacts\n";
	broadPhase.UpdatePairs(this);
}
} // namespace ale