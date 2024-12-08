#include "physics/Contact.h"

namespace ale
{
Contact *Contact::create(Fixture *fixtureA, int32_t indexA, Fixture *fixtureB, int32_t indexB)
{
}
void Contact::update()
{
	// 기존 manifold 저장
	Manifold oldManifold = m_manifold;

	// 이전 프레임에서 두 객체가 충돌중이었는지 확인
	bool touching = false;
	bool wasTouching = (m_flags & EContactFlag::TOUCHING) == EContactFlag::TOUCHING;

	// bodyA, bodyB의 Transform 가져오기
	Rigidbody *bodyA = m_fixtureA->getBody();
	Rigidbody *bodyB = m_fixtureB->getBody();
	const Transform &transformA = bodyA->getTransform();
	const Transform &transformB = bodyB->getTransform();

	// Evaluate
	// 두 shape의 변환 상태를 적용해 world space에서의 충돌 정보를 계산
	// 1. 두 도형이 실제로 충돌하는지 검사
	// 2. 충돌에 따른 manifold 생성
	// 3. manifold의 내부 값을 impulse를 제외하고 채워줌
	// 4. 실제 충돌이 일어나지 않은 경우 manifold.pointCount = 0인 충돌 생성
	evaluate(&m_manifold, transformA, transformB);
	touching = m_manifold.pointCount > 0;

	// manifold의 충격량 0으로 초기화 및 old manifold 중
	// 같은 충돌이 있는경우 Impulse 재사용
	// id 는 충돌 도형의 type과 vertex 또는 line의 index 정보를 압축하여 결정
	for (int32_t i = 0; i < m_manifold.pointCount; ++i)
	{
		ManifoldPoint *manifoldPoint = m_manifold.points + i;
		manifoldPoint->normalImpulse = 0.0f;
		manifoldPoint->tangentImpulse = 0.0f;
		uint32_t manifoldId = manifoldPoint->id;

		for (int32_t j = 0; j < oldManifold.pointCount; ++j)
		{
			ManifoldPoint *oldManifoldPoint = oldManifold.points + j;

			// oldmanifold에 똑같은 manifold가 존재하는 경우 impulse 덮어쓰기
			if (oldManifoldPoint->id == manifoldId)
			{
				manifoldPoint->normalImpulse = oldManifoldPoint->normalImpulse;
				manifoldPoint->tangentImpulse = oldManifoldPoint->tangentImpulse;
				break;
			}
		}
	}

	if (touching)
	{
		// touching시 touching flag on
		m_flags |= EContactFlag::TOUCHING;
	}
	else
	{
		// touching이 아니면 touching flag off
		m_flags &= ~EContactFlag::TOUCHING;
	}
}

inline Contact *Contact::getNext()
{
	return m_next;
}

inline Fixture *Contact::getFixtureA() const
{
	return m_fixtureA;
}

inline Fixture *Contact::getFixtureB() const
{
	return m_fixtureB;
}

inline int32_t Contact::getChildIndexA() const
{
	return m_indexA;
}

inline int32_t Contact::getChildIndexB() const
{
	return m_indexB;
}
} // namespace ale