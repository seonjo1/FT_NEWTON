#include "physics/ContactSolver.h"

namespace ale
{

const float ContactSolver::NORMAL_STOP_VELOCITY = 0.001f;
const float ContactSolver::TANGENT_STOP_VELOCITY = 0.001f;
const float ContactSolver::NORMAL_SLEEP_VELOCITY = 1.0f;
const float ContactSolver::TANGENT_SLEEP_VELOCITY = 1.0f;

ContactSolver::ContactSolver(float duration, Contact **contacts, Position *positions, Velocity *velocities,
							 int32_t bodyCount, int32_t contactCount)
	: m_duration(duration), m_positions(positions), m_velocities(velocities), m_contacts(contacts),
	  m_bodyCount(bodyCount), m_contactCount(contactCount)
{
	// std::cout << "ContactSolver Constructor\n";
	// std::cout << "constactCount: " << contactCount << "\n";
	m_positionConstraints = static_cast<ContactPositionConstraint *>(
		PhysicsAllocator::m_stackAllocator.allocateStack(sizeof(ContactPositionConstraint) * contactCount));
	m_velocityConstraints = static_cast<ContactVelocityConstraint *>(
		PhysicsAllocator::m_stackAllocator.allocateStack(sizeof(ContactVelocityConstraint) * contactCount));

	for (int32_t i = 0; i < contactCount; i++)
	{
		Contact *contact = m_contacts[i];

		// contact 정보 가져오기
		Fixture *fixtureA = contact->getFixtureA();
		Fixture *fixtureB = contact->getFixtureB();
		Shape *shapeA = fixtureA->getShape();
		Shape *shapeB = fixtureB->getShape();
		Rigidbody *bodyA = fixtureA->getBody();
		Rigidbody *bodyB = fixtureB->getBody();
		Manifold &manifold = contact->getManifold();

		new (m_velocityConstraints + i) ContactVelocityConstraint();
		new (m_positionConstraints + i) ContactPositionConstraint();

		// 속도 제약 설정
		m_velocityConstraints[i].friction = contact->getFriction();
		m_velocityConstraints[i].restitution = contact->getRestitution();
		m_velocityConstraints[i].worldCenterA = bodyA->getTransform().toMatrix() * glm::vec4(shapeA->m_center, 1.0f);
		m_velocityConstraints[i].worldCenterB = bodyB->getTransform().toMatrix() * glm::vec4(shapeB->m_center, 1.0f);
		m_velocityConstraints[i].indexA = bodyA->getIslandIndex();
		m_velocityConstraints[i].indexB = bodyB->getIslandIndex();
		m_velocityConstraints[i].invMassA = bodyA->getInverseMass();
		m_velocityConstraints[i].invMassB = bodyB->getInverseMass();
		m_velocityConstraints[i].invIA = bodyA->getInverseInertiaTensorWorld();
		m_velocityConstraints[i].invIB = bodyB->getInverseInertiaTensorWorld();
		m_velocityConstraints[i].pointCount = manifold.pointsCount;
		m_velocityConstraints[i].points = manifold.points;

		// 위치 제약 설정
		m_positionConstraints[i].worldCenterA = bodyA->getTransform().toMatrix() * glm::vec4(shapeA->m_center, 1.0f);
		m_positionConstraints[i].worldCenterB = bodyB->getTransform().toMatrix() * glm::vec4(shapeB->m_center, 1.0f);
		m_positionConstraints[i].indexA = bodyA->getIslandIndex();
		m_positionConstraints[i].indexB = bodyB->getIslandIndex();
		m_positionConstraints[i].invMassA = bodyA->getInverseMass();
		m_positionConstraints[i].invMassB = bodyB->getInverseMass();
		m_velocityConstraints[i].invIA = bodyA->getInverseInertiaTensorWorld();
		m_velocityConstraints[i].invIB = bodyB->getInverseInertiaTensorWorld();
		m_positionConstraints[i].resolvedSeperation = 0.0f;
		m_positionConstraints[i].pointCount = manifold.pointsCount;
		m_positionConstraints[i].points = manifold.points;
	}
}

void ContactSolver::destroy()
{
	for (int32_t i = 0; i < m_contactCount; i++)
	{
		m_positionConstraints[i].~ContactPositionConstraint();
		m_velocityConstraints[i].~ContactVelocityConstraint();
	}

	PhysicsAllocator::m_stackAllocator.freeStack();
	PhysicsAllocator::m_stackAllocator.freeStack();
}

void ContactSolver::solveVelocityConstraints()
{
	for (int32_t i = 0; i < m_contactCount; i++)
	{
		Contact *contact = m_contacts[i];
		ContactVelocityConstraint &velocityConstraint = m_velocityConstraints[i];
		int32_t pointCount = velocityConstraint.pointCount;
		int32_t indexA = velocityConstraint.indexA;
		int32_t indexB = velocityConstraint.indexB;

		glm::vec3 &linearVelocityA = m_velocities[indexA].linearVelocity;
		glm::vec3 &linearVelocityB = m_velocities[indexB].linearVelocity;
		glm::vec3 &angularVelocityA = m_velocities[indexA].angularVelocity;
		glm::vec3 &angularVelocityB = m_velocities[indexB].angularVelocity;

		bool isStop = true;

		for (int32_t j = 0; j < pointCount; j++)
		{
			ManifoldPoint &manifoldPoint = velocityConstraint.points[j];

			glm::vec3 rA = manifoldPoint.pointA - velocityConstraint.worldCenterA; // bodyA의 충돌 지점까지의 벡터
			glm::vec3 rB = manifoldPoint.pointB - velocityConstraint.worldCenterB; // bodyB의 충돌 지점까지의 벡터

			// 상대 속도 계산
			glm::vec3 velocityA = linearVelocityA + glm::cross(angularVelocityA, rA);
			glm::vec3 velocityB = linearVelocityB + glm::cross(angularVelocityB, rB);
			glm::vec3 relativeVelocity = velocityB - velocityA;

			// 법선 방향 속도

			float normalSpeed = glm::dot(relativeVelocity, manifoldPoint.normal);

			if (normalSpeed < 0.0f)
			{
				isStop = false;

				float oldNormalImpulse = manifoldPoint.normalImpulse;

				float appliedNormalImpulse = -(1.0f + velocityConstraint.restitution) * normalSpeed;
				float inverseMasses = (velocityConstraint.invMassA + velocityConstraint.invMassB);
				float normalEffectiveMassA = glm::dot(glm::cross(manifoldPoint.normal, rA),
													  velocityConstraint.invIA * glm::cross(manifoldPoint.normal, rA));
				float normalEffectiveMassB = glm::dot(glm::cross(manifoldPoint.normal, rB),
													  velocityConstraint.invIB * glm::cross(manifoldPoint.normal, rB));

				appliedNormalImpulse =
					appliedNormalImpulse / (inverseMasses + normalEffectiveMassA + normalEffectiveMassB);

				float newNormalImpulse = appliedNormalImpulse + oldNormalImpulse;

				if (newNormalImpulse < 0.0f)
				{
					newNormalImpulse = 0.0f;
				}

				manifoldPoint.normalImpulse = newNormalImpulse;

				linearVelocityA -= velocityConstraint.invMassA * appliedNormalImpulse * manifoldPoint.normal;
				linearVelocityB += velocityConstraint.invMassB * appliedNormalImpulse * manifoldPoint.normal;
				angularVelocityA -=
					velocityConstraint.invIA * glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal);
				angularVelocityB +=
					velocityConstraint.invIB * glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal);
			}

			velocityA = linearVelocityA + glm::cross(angularVelocityA, rA);
			velocityB = linearVelocityB + glm::cross(angularVelocityB, rB);
			relativeVelocity = velocityB - velocityA;

			// 접선 방향 충격량 계산
			normalSpeed = glm::dot(relativeVelocity, manifoldPoint.normal);
			glm::vec3 tangentVelocity = relativeVelocity - (normalSpeed * manifoldPoint.normal);
			float tangentSpeed = glm::length(tangentVelocity);
			if (tangentSpeed > 1e-6f)
			{
				isStop = false;

				glm::vec3 tangent = glm::normalize(tangentVelocity);

				float oldTangentImpulse = manifoldPoint.tangentImpulse;
				float newTangentImpulse = glm::dot(relativeVelocity, tangent);

				float inverseMasses = (velocityConstraint.invMassA + velocityConstraint.invMassB);
				float tangentEffectiveMassA =
					glm::dot(glm::cross(tangent, rA), velocityConstraint.invIA * glm::cross(tangent, rA));
				float tangentEffectiveMassB =
					glm::dot(glm::cross(tangent, rB), velocityConstraint.invIB * glm::cross(tangent, rB));
				newTangentImpulse = newTangentImpulse / (inverseMasses + tangentEffectiveMassA + tangentEffectiveMassB);
				newTangentImpulse += oldTangentImpulse;

				float maxFriction = velocityConstraint.friction * manifoldPoint.normalImpulse;
				newTangentImpulse = glm::clamp(newTangentImpulse, -maxFriction, maxFriction);

				manifoldPoint.tangentImpulse = newTangentImpulse;

				float appliedTangentImpulse = newTangentImpulse - oldTangentImpulse;

				linearVelocityA += velocityConstraint.invMassA * appliedTangentImpulse * tangent;
				linearVelocityB -= velocityConstraint.invMassB * appliedTangentImpulse * tangent;
				angularVelocityA += velocityConstraint.invIA * glm::cross(rA, appliedTangentImpulse * tangent);
				angularVelocityB -= velocityConstraint.invIB * glm::cross(rB, appliedTangentImpulse * tangent);
			}

			if (std::abs(normalSpeed) > NORMAL_SLEEP_VELOCITY || tangentSpeed > TANGENT_SLEEP_VELOCITY)
			{
				isStop = false;
			}
		}

		if (isStop == false)
		{
			m_positions[indexA].isStop = false;
			m_positions[indexB].isStop = false;
		}
	}
}

void ContactSolver::solvePositionConstraints()
{
	const float kSlop = 0.001f; // 허용 관통 오차
	const float alpha = 0.2f;

	for (int i = 0; i < m_contactCount; ++i)
	{
		Contact *contact = m_contacts[i];
		ContactPositionConstraint &positionConstraint = m_positionConstraints[i];
	
		int32_t pointCount = positionConstraint.pointCount;
		int32_t indexA = positionConstraint.indexA;
		int32_t indexB = positionConstraint.indexB;

		float &resolvedSeperation = positionConstraint.resolvedSeperation;

		glm::mat3 &invIA = positionConstraint.invIA;
		glm::mat3 &invIB = positionConstraint.invIB;
	
		float sumMass = positionConstraint.invMassA + positionConstraint.invMassB;
		float ratioA = positionConstraint.invMassA / sumMass;
		float ratioB = positionConstraint.invMassB / sumMass;

		glm::vec3 &positionA = m_positions[indexA].position;
		glm::vec3 &positionB = m_positions[indexB].position;
		glm::quat &orientationA = m_positions[indexA].orientation;
		glm::quat &orientationB = m_positions[indexB].orientation;

		for (int32_t j = 0; j < pointCount; j++)
		{

			ManifoldPoint &manifoldPoint = positionConstraint.points[j];

			if (manifoldPoint.seperation <= 0.0f)
			{
				continue;
			}

			// glm::vec3 &pointA = manifoldPoint.pointA;
			// glm::vec3 &pointB = manifoldPoint.pointB;

			float seperation = manifoldPoint.seperation - resolvedSeperation;

			// 관통 해소된상태면 무시
			if (seperation < kSlop)
			{
				continue;
			}
			
			// 관통 깊이에 따른 보정량 계산
			float correction = seperation * alpha;
			glm::vec3 correctionVector = correction * manifoldPoint.normal;

			resolvedSeperation += correction;

			positionA -= correctionVector * ratioA;
			positionB += correctionVector * ratioB;

			// 회전 보정

			glm::vec3 rA = (manifoldPoint.pointA - manifoldPoint.normal * resolvedSeperation) - positionConstraint.worldCenterA;
			glm::vec3 rB = (manifoldPoint.pointB + manifoldPoint.normal * resolvedSeperation) - positionConstraint.worldCenterB;

			glm::vec3 angularVelocityA =  positionConstraint.invIA * glm::cross(rA, correctionVector * ratioA);
			glm::vec3 angularVelocityB =  positionConstraint.invIB * glm::cross(rB, correctionVector * ratioB);
			
			glm::quat angularVelocityQuat = glm::quat(0.0f, angularVelocityA);
			orientationA += 0.5f * angularVelocityQuat * orientationA;
			orientationA = glm::normalize(orientationA);

			angularVelocityQuat = glm::quat(0.0f, angularVelocityB);
			orientationB += 0.5f * angularVelocityQuat * orientationB;
			orientationB = glm::normalize(orientationB);

		}
	}
}

} // namespace ale
