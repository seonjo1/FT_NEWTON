#ifndef CONTACT_H
#define CONTACT_H

#include "Rigidbody.h"

namespace ale
{

class Contact;
class Manifold;

struct ContactLink
{
	Rigidbody *other;  // 연결된 반대쪽 Body
	Contact *contact;  // 두 Body 간의 Contact 정보
	ContactLink *prev; // 이전 충돌 정보
	ContactLink *next; // 다음 충돌 정보
};

enum class EContactFlag
{
	ISLAND = 0x0001,
	TOUCHING = 0x0002,

	// e_enabledFlag		= 0x0004,

	// // This contact needs filtering because a fixture filter was changed.
	// e_filterFlag		= 0x0008,

	// // This bullet contact had a TOI event
	// e_bulletHitFlag		= 0x0010,

	// // This contact has a valid TOI in m_toi
	// e_toiFlag			= 0x0020
};

class Contact
{
  public:
	Contact *create(Fixture *fixtureA, int32_t indexA, Fixture *fixtureB, int32_t indexB);
	void update();
	virtual void evaluate(Manifold *manifold, const Transform &transformA, const Transform &transformB) = 0;

	Contact *getNext();
	Fixture *getFixtureA() const;
	Fixture *getFixtureB() const;
	int32_t getChildIndexA() const;
	int32_t getChildIndexB() const;

  protected:
	uint32_t m_flags;

	Contact *m_prev;
	Contact *m_next;

	ContactLink m_nodeA;
	ContactLink m_nodeB;

	Fixture *m_fixtureA;
	Fixture *m_fixtureB;

	int32_t m_indexA;
	int32_t m_indexB;

	Manifold m_manifold;

	int32_t m_toiCount;
	float m_toi;

	float m_friction;
	float m_restitution;

	float m_tangentSpeed;
};
} // namespace ale

#endif