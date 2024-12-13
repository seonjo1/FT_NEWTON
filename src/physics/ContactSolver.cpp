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
		Fixture *fixtureA = contact->getFixtureA();
		Fixture *fixtureB = contact->getFixtureB();
		Shape *shapeA = fixtureA->getShape();
		Shape *shapeB = fixtureB->getShape();
		float radiusA = shapeA->getLocalRadius();
		float radiusB = shapeB->getLocalRadius();
		Rigidbody *bodyA = fixtureA->getBody();
		Rigidbody *bodyB = fixtureB->getBody();
		Manifold manifold = contact->getManifold();

		// 속도 제약 설정
		ContactVelocityConstraint velocityConstraint;
		velocityConstraint.friction = contact->getFriction();
		velocityConstraint.restitution = contact->getRestitution();
		velocityConstraint.tangentSpeed = contact->getTangentSpeed();
		velocityConstraint.indexA = bodyA->getIslandIndex();
		velocityConstraint.indexB = bodyB->getIslandIndex();
		velocityConstraint.invMassA = bodyA->getInverseMass();
		velocityConstraint.invMassB = bodyB->getInverseMass();
		velocityConstraint.invIA = bodyA->getInverseInertiaTensorWorld();
		velocityConstraint.invIB = bodyB->getInverseInertiaTensorWorld();
		velocityConstraint.contactIndex = i;
		velocityConstraint.pointCount = manifold.points.size();
		velocityConstraint.points = manifold.points.data();
		velocityConstraint.K = glm::mat3(0.0f);
		velocityConstraint.normalMass = glm::mat3(0.0f);


		// 위치 제약 설정
		ContactPositionConstraint positionConstraint;
		positionConstraint.indexA = bodyA->getIslandIndex();
		positionConstraint.indexB = bodyB->getIslandIndex();
		positionConstraint.invMassA = bodyA->getInverseMass();
		positionConstraint.invMassB = bodyB->getInverseMass();
		positionConstraint.localCenterA = contact->getFixtureA()->getShape()->localCenter;
		positionConstraint.localCenterB = contact->getFixtureB()->getShape()->localCenter;
		positionConstraint.invIA = bodyA->getInverseInertiaTensorWorld();
		positionConstraint.invIB = bodyB->getInverseInertiaTensorWorld();
		positionConstraint.pointCount = manifold.points.size();
		positionConstraint.points = manifold.points.data();


		// // manifold의 points 순회
		// int32_t pointSize = manifold.points.size();
		// velocityConstraint.points.resize(pointSize);
		// for (int j = 0; j < pointSize; j++)
		// {
		// 	ManifoldPoint &manifoldPoint = manifold.points[j];
		// 	// 속도 제약 설정의 point 설정
		// 	VelocityConstraintPoint &velocityPoint = velocityConstraint->points[j];

		// 	// warmStarting인 경우 초기 impulse 설정
		// 	if (m_step.warmStarting)
		// 	{
		// 		velocityPoint.normalImpulse = m_step.dtRatio * manifoldPoint.normalImpulse;
		// 		velocityPoint.tangentImpulse = m_step.dtRatio * manifoldPoint.tangentImpulse;
		// 	}
		// 	else
		// 	{
		// 		velocityPoint.normalImpulse = 0.0f;
		// 		velocityPoint.tangentImpulse = 0.0f;
		// 	}

		// 	velocityPoint.rA.SetZero();
		// 	velocityPoint.rB.SetZero();
		// 	velocityPoint.normalMass = 0.0f;
		// 	velocityPoint.tangentMass = 0.0f;
		// 	velocityPoint.velocityBias = 0.0f;

		// 	// 위치 제약 설정의 point 설정
		// 	positionConstraint.localPoints[j] = manifoldPoint.localPoint;
		// }

		m_velocityConstraints.push_back(velocityConstraint);
		m_positionConstraints.push_back(positionConstraint);
	}
}

void ContactSolver::initializeVelocityConstraints()
{
}

} // namespace ale
