#include "physics/ContactManager.h"
#include "physics/Fixture.h"

namespace ale
{
ContactManager::ContactManager()
{
}

bool ContactManager::isSameContact(ContactLink *link, Fixture *fixtureA, Fixture *fixtureB, int32_t indexA,
								   int32_t indexB)
{
	Fixture *fixtureX = link->contact->GetFixtureA();
	Fixture *fixtureY = link->contact->GetFixtureB();
	int32_t indexX = link->contact->GetChildIndexA();
	int32_t indexY = link->contact->GetChildIndexB();

	// 같은 충돌인 경우 충돌 생성 x
	if (fixtureX == fixtureA && fixtureY == fixtureB && indexX == indexA && indexY == indexB)
	{
		return true;
	}
	if (fixtureX == fixtureB && fixtureY == fixtureA && indexX == indexB && indexY == indexA)
	{
		return true;
	}

	return false;
}

void ContactManager::addPair(void *proxyUserDataA, void *proxyUserDataB)
{
	FixtureProxy *proxyA = static_cast<FixtureProxy *>(proxyUserDataA);
	FixtureProxy *proxyB = static_cast<FixtureProxy *>(proxyUserDataB);

	Fixture *fixtureA = proxyA->fixture;
	Fixture *fixtureB = proxyB->fixture;

	int32_t indexA = proxyA->childIndex;
	int32_t indexB = proxyB->childIndex;

	Rigidbody *bodyA = fixtureA->getBody();
	Rigidbody *bodyB = fixtureB->getBody();

	// 같은 Body간 충돌인 경우 return
	if (bodyA == bodyB)
	{
		return;
	}

	// 동일 부위의 충돌이 있는 경우 return
	ContactLink *link = bodyA->GetContactLinks();
	while (link)
	{
		if (link->other == bodyB && isSameContact(link, fixtureA, fixtureB, indexA, indexB))
		{
			return;
		}
		link = link->next;
	}

	// BodyA와 BodyB가 충돌 가능 관계인지 확인
	if (bodyA->ShouldCollide(bodyB) == false)
	{
		return;
	}

	// 충돌 생성
	Contact* contact = Contact::create(fixtureA, indexA, fixtureB, indexB);
	if (contact == nullptr)
	{
		return;
	}

	fixtureA = contact->getFixtureA();
	fixtureB = contact->getFixtureB();
	indexA = contact->getChildIndexA();
	indexB = contact->getChildIndexB();
	bodyA = fixtureA->getBody();
	bodyB = fixtureB->getBody();

	// contact를 world contactList 앞에 끼워넣기 (Contact*)
	contact->m_prev = nullptr;
	contact->m_next = m_contactList;
	if (m_contactList != nullptr)
	{
		m_contactList->m_prev = contact;
	}
	m_contactList = contact;

	// contact의 m_nodeA 초기화
	contact->m_nodeA.contact = contact;
	contact->m_nodeA.other = bodyB;

	// bodyA의 contactLinks에 새로운 Link 추가
	contact->m_nodeA.prev = nullptr;
	contact->m_nodeA.next = bodyA->m_contactLinks;
	if (bodyA->m_contactLinks != nullptr)
	{
		bodyA->m_contactLinks->prev = &contact->m_nodeA;
	}
	bodyA->m_contactLinks = &contact->m_nodeA;

	// contact의 m_nodeB 초기화
	contact->m_nodeB.contact = contact;
	contact->m_nodeB.other = bodyA;

	// bodyB의 contactLinks에 새로운 Link 추가
	contact->m_nodeB.prev = nullptr;
	contact->m_nodeB.next = bodyB->m_contactLinks;
	if (bodyB->m_contactLinks != nullptr)
	{
		bodyB->m_contactLinks->prev = &contact->m_nodeB;
	}
	bodyB->m_contactLinks = &contact->m_nodeB;

	// // body Awake 상태 설정
	// bodyA->SetAwake(true);
	// bodyB->SetAwake(true);

	// 충돌 개수 추가
	++m_contactCount;
}

void ContactManager::findNewContacts()
{
	// std::cout << "ContactManager::findNewContacts\n";
	broadPhase.UpdatePairs(this);
}
} // namespace ale