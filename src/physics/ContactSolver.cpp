#include "ContactSolver.h"

namespace ale
{
ContactSolver::ContactSolver(float duration, std::vector<Contact *> &contacts, std::vector<Position> &positions,
							 std::vector<Velocity> &velocities)
	: m_duration(duration), m_positions(positions), m_velocities(velocities), m_contacts(contacts)
{
	int32_t size = contacts.size();
	for (int32_t i = 0; i < size; i++)
	{
		Contact *contact = m_contacts[i];

		// contact 정보 가져오기
		Fixture *fixtureA = contact->m_fixtureA;
		Fixture *fixtureB = contact->m_fixtureB;
		Shape *shapeA = fixtureA->getShape();
		Shape *shapeB = fixtureB->getShape();
		float radiusA = shapeA->m_radius;
		float radiusB = shapeB->m_radius;
		Rigidbody *bodyA = fixtureA->getBody();
		Rigidbody *bodyB = fixtureB->getBody();
		Manifold *manifold = contact->getManifold();

		// 속도 제약 설정
		ContactVelocityConstraint velocityConstraint;
		velocityConstraint.friction = contact->m_friction;
		velocityConstraint.restitution = contact->m_restitution;
		velocityConstraint.tangentSpeed = contact->m_tangentSpeed;
		velocityConstraint.indexA = bodyA->m_islandIndex;
		velocityConstraint.indexB = bodyB->m_islandIndex;
		velocityConstraint.invMassA = bodyA->m_invMass;
		velocityConstraint.invMassB = bodyB->m_invMass;
		velocityConstraint.invIA = bodyA->m_invI;
		velocityConstraint.invIB = bodyB->m_invI;
		velocityConstraint.contactIndex = i;
		velocityConstraint.pointCount = pointCount;
		velocityConstraint.K.SetZero();
		velocityConstraint.normalMass.SetZero();

		m_velocityConstraints.push_back(velocityConstraint);

		// 위치 제약 설정
		ContactPositionConstraint positionConstraint;
		positionConstraint.indexA = bodyA->m_islandIndex;
		positionConstraint.indexB = bodyB->m_islandIndex;
		positionConstraint.invMassA = bodyA->m_invMass;
		positionConstraint.invMassB = bodyB->m_invMass;
		positionConstraint.localCenterA = bodyA->m_sweep.localCenter;
		positionConstraint.localCenterB = bodyB->m_sweep.localCenter;
		positionConstraint.invIA = bodyA->m_invI;
		positionConstraint.invIB = bodyB->m_invI;
		positionConstraint.localNormal = manifold->localNormal;
		positionConstraint.localPoint = manifold->localPoint;
		positionConstraint.pointCount = pointCount;
		positionConstraint.radiusA = radiusA;
		positionConstraint.radiusB = radiusB;
		positionConstraint.type = manifold->type;

		m_positionConstraints.push_back(positionConstraint);

		// manifold의 points 순회
		int32_t pointSize = manifold->points.size();
		for (int j = 0; j < pointSize; j++)
		{
			ManifoldPoint &manifoldPoint = manifold->points[j];
			// 속도 제약 설정의 point 설정
			VelocityConstraintPoint &velocityPoint = velocityConstraint->points[j];

			// warmStarting인 경우 초기 impulse 설정
			if (m_step.warmStarting)
			{
				velocityPoint.normalImpulse = m_step.dtRatio * manifoldPoint.normalImpulse;
				velocityPoint.tangentImpulse = m_step.dtRatio * manifoldPoint.tangentImpulse;
			}
			else
			{
				velocityPoint.normalImpulse = 0.0f;
				velocityPoint.tangentImpulse = 0.0f;
			}

			velocityPoint.rA.SetZero();
			velocityPoint.rB.SetZero();
			velocityPoint.normalMass = 0.0f;
			velocityPoint.tangentMass = 0.0f;
			velocityPoint.velocityBias = 0.0f;

			// 위치 제약 설정의 point 설정
			positionConstraint.localPoints[j] = manifoldPoint.localPoint;
		}
	}
}

void ContactSolver::initializeVelocityConstraints()
{
}

} // namespace ale
