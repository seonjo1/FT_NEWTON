#include "physics/ContactManager.h"
#include "physics/Fixture.h"

namespace ale
{
ContactManager::ContactManager()
{
	m_contactList = nullptr;
}

bool ContactManager::isSameContact(ContactLink *link, Fixture *fixtureA, Fixture *fixtureB, int32_t indexA,
								   int32_t indexB)
{
	Fixture *fixtureX = link->contact->getFixtureA();
	Fixture *fixtureY = link->contact->getFixtureB();
	int32_t indexX = link->contact->getChildIndexA();
	int32_t indexY = link->contact->getChildIndexB();

	// 같은 충돌인 경우 충돌 생성 x
	if (fixtureX == fixtureA && fixtureY == fixtureB && indexX == indexA && indexY == indexB)
	{
		link->contact->setFlag(EContactFlag::TOUCHING);
		return true;
	}
	if (fixtureX == fixtureB && fixtureY == fixtureA && indexX == indexB && indexY == indexA)
	{
		link->contact->setFlag(EContactFlag::TOUCHING);
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

	// std::cout << "addPair\n" << "bodyA: " << bodyA->getBodyId() << "\nbodyB: " << bodyB->getBodyId() << "\n";

	// 같은 Body간 충돌인 경우 return
	if (bodyA == bodyB)
	{
		return;
	}

	// 동일 부위의 충돌이 있는 경우 return
	ContactLink *link = bodyA->getContactLinks();
	while (link)
	{
		if (link->other == bodyB && isSameContact(link, fixtureA, fixtureB, indexA, indexB))
		{
			return;
		}
		link = link->next;
	}

	// BodyA와 BodyB가 충돌 가능 관계인지 확인
	if (bodyA->shouldCollide(bodyB) == false)
	{
		return;
	}
	// 충돌 생성
	Contact *contact = Contact::create(fixtureA, fixtureB, indexA, indexB);
	if (contact == nullptr)
	{
		throw std::runtime_error("Generating Contact fail!");
	}
	// std::cout << "create Contact\n" << "bodyA: " << bodyA->getBodyId() << "\nbodyB: " << bodyB->getBodyId() << "\n";

	fixtureA = contact->getFixtureA();
	fixtureB = contact->getFixtureB();
	indexA = contact->getChildIndexA();
	indexB = contact->getChildIndexB();
	bodyA = fixtureA->getBody();
	bodyB = fixtureB->getBody();

	// contact를 world contactList 앞에 끼워넣기 (Contact*)
	contact->setNext(m_contactList);
	if (m_contactList != nullptr)
	{
		m_contactList->setPrev(contact);
	}
	m_contactList = contact;

	// contact의 m_nodeA 초기화
	ContactLink *nodeA = contact->getNodeA();
	ContactLink *bodyAContactLinks = bodyA->getContactLinks();

	nodeA->contact = contact;
	nodeA->other = bodyB;

	// bodyA의 contactLinks에 새로운 Link 추가
	nodeA->next = bodyAContactLinks;
	if (bodyAContactLinks != nullptr)
	{
		bodyAContactLinks->prev = nodeA;
	}
	bodyA->setContactLinks(nodeA);

	// contact의 m_nodeB 초기화
	ContactLink *nodeB = contact->getNodeB();
	ContactLink *bodyBContactLinks = bodyB->getContactLinks();

	nodeB->contact = contact;
	nodeB->other = bodyA;

	// bodyB의 contactLinks에 새로운 Link 추가
	nodeB->next = bodyBContactLinks;
	if (bodyBContactLinks != nullptr)
	{
		bodyBContactLinks->prev = nodeB;
	}
	bodyB->setContactLinks(nodeB);
}

void ContactManager::findNewContacts()
{
	// std::cout << "ContactManager::findNewContacts\n";
	broadPhase.updatePairs(this);
}

void ContactManager::collide()
{
	Contact *contact = m_contactList;

	// contactList 순회
	while (contact)
	{
		// 실제 충돌 여부를 검사하고 해당 충돌 정보인 manifold 생성
		if (contact->hasFlag(EContactFlag::TOUCHING))
		{
			contact->update();
		}
		contact = contact->getNext();
	}
}

} // namespace ale
