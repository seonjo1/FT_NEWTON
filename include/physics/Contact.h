#ifndef CONTACT_H
#define CONTACT_H

#include "Fixture.h"
#include <cmath>

namespace ale
{

class Contact;
struct Manifold;

using contactMemberFunction = Contact *(*)(Fixture *, Fixture *, int32_t, int32_t);

struct ContactLink
{
	Rigidbody *other;  // 연결된 반대쪽 Body
	Contact *contact;  // 두 Body 간의 Contact 정보
	ContactLink *prev; // 이전 충돌 정보
	ContactLink *next; // 다음 충돌 정보

	ContactLink() : other(nullptr), contact(nullptr), prev(nullptr), next(nullptr) {};
};

enum class EContactType
{
	SPHERE_TO_SPHERE = (1 << 0),
	BOX_TO_BOX = (1 << 1),
	SphereToBox = (1 << 0) | (1 << 1),
};

enum class EContactFlag
{
	ISLAND = (1 << 0),
	TOUCHING = (1 << 2),
};

int32_t operator&(int32_t val, EContactFlag flag);
int32_t operator|(int32_t val, EContactFlag flag);
int32_t operator~(EContactFlag flag);
bool operator==(int32_t val, EContactFlag flag);

class Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	Contact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	void update();
	virtual void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB) = 0;

	float getFriction() const;
	float getRestitution() const;
	float getTangentSpeed() const;
	Contact *getNext();
	Fixture *getFixtureA() const;
	Fixture *getFixtureB() const;
	int32_t getChildIndexA() const;
	int32_t getChildIndexB() const;
	ContactLink *getNodeA();
	ContactLink *getNodeB();
	const Manifold &getManifold() const;

	void setPrev(Contact *contact);
	void setNext(Contact *contact);
	void setFlag(EContactFlag flag);
	void unsetFlag(EContactFlag flag);
	bool hasFlag(EContactFlag flag);

  protected:
	static contactMemberFunction createContactFunctions[8];

	int32_t m_flags;

	Contact *m_prev;
	Contact *m_next;

	ContactLink m_nodeA;
	ContactLink m_nodeB;

	Fixture *m_fixtureA;
	Fixture *m_fixtureB;

	int32_t m_indexA;
	int32_t m_indexB;

	Manifold m_manifold;

	float m_friction;
	float m_restitution;

	float m_tangentSpeed;
};
} // namespace ale

#endif