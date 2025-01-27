#include "physics/ContactSolver.h"

namespace ale
{

const float ContactSolver::NORMAL_STOP_VELOCITY = 0.1f;
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

		// std::cout << "\n\nbefore angular velocity!!!\n";
		// std::cout << "linear velocityA: " << linearVelocityA.x << " " << linearVelocityA.y << " " << linearVelocityA.z << "\n";
		// std::cout << "linear velocityB: " << linearVelocityB.x << " " << linearVelocityB.y << " " << linearVelocityB.z << "\n";
		// std::cout << "angular velocityA: " << angularVelocityA.x << " " << angularVelocityA.y << " " << angularVelocityA.z << "\n";
		// std::cout << "angular velocityB: " << angularVelocityB.x << " " << angularVelocityB.y << " " << angularVelocityB.z << "\n";

		glm::vec3 &linearVelocityBufferA = m_velocities[indexA].linearVelocityBuffer;
		glm::vec3 &linearVelocityBufferB = m_velocities[indexB].linearVelocityBuffer;
		glm::vec3 &angularVelocityBufferA = m_velocities[indexA].angularVelocityBuffer;
		glm::vec3 &angularVelocityBufferB = m_velocities[indexB].angularVelocityBuffer;

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
			float appliedNormalImpulse;

			if (normalSpeed < NORMAL_STOP_VELOCITY)
			{
				// std::cout << "normal in!!\n";
				isStop = false;

				float oldNormalImpulse = manifoldPoint.normalImpulse;
				appliedNormalImpulse = -(1.0f + velocityConstraint.restitution) * normalSpeed / pointCount;
				float inverseMasses = (velocityConstraint.invMassA + velocityConstraint.invMassB);
				float normalEffectiveMassA = glm::dot(glm::cross(manifoldPoint.normal, rA),
													  velocityConstraint.invIA * glm::cross(manifoldPoint.normal, rA));
				float normalEffectiveMassB = glm::dot(glm::cross(manifoldPoint.normal, rB),
													  velocityConstraint.invIB * glm::cross(manifoldPoint.normal, rB));

				appliedNormalImpulse =
					appliedNormalImpulse / (inverseMasses + normalEffectiveMassA + normalEffectiveMassB);

				float newNormalImpulse = appliedNormalImpulse + oldNormalImpulse;

				manifoldPoint.normalImpulse = newNormalImpulse;

				if (manifoldPoint.normalImpulse <= 0.0f)
				{
					manifoldPoint.normalImpulse = 0.0f;
				}

				linearVelocityBufferA -= velocityConstraint.invMassA * appliedNormalImpulse * manifoldPoint.normal;
				linearVelocityBufferB += velocityConstraint.invMassB * appliedNormalImpulse * manifoldPoint.normal;

				angularVelocityBufferA -=
					velocityConstraint.invIA * glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal);
				angularVelocityBufferB +=
					velocityConstraint.invIB * glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal);

				// std::cout << "\n\nAAAA\n";
				
				// std::cout << "before: " << glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal).x << " "
				// 		  << glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal).y << " "
				// 		  << glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal).z << "\n";
						  
				// std::cout << "after: " << (velocityConstraint.invIA * glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal)).x << " "
				// 		  << (velocityConstraint.invIA * glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal)).y << " "
				// 		  << (velocityConstraint.invIA * glm::cross(rA, appliedNormalImpulse * manifoldPoint.normal)).z << "\n";
				
				// std::cout << "\n\nBBB\n";
				// std::cout << "before: " << glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal).x << " "
				// 		  << glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal).y << " "
				// 		  << glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal).z << "\n";
						  
				// std::cout << "after: " << (velocityConstraint.invIB * glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal)).x << " "
				// 		  << (velocityConstraint.invIB * glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal)).y << " "
				// 		  << (velocityConstraint.invIB * glm::cross(rB, appliedNormalImpulse * manifoldPoint.normal)).z << "\n";
			}
			else if (normalSpeed > NORMAL_SLEEP_VELOCITY)
			{
				// std::cout << "\n\nnormal stop!!\n";
				isStop = false;
			}
			else
			{
				// std::cout << "\n\nnormal stop!!\n";
			}

			// 접선 방향 충격량 계산
			glm::vec3 tangentVelocity = relativeVelocity - (normalSpeed * manifoldPoint.normal);
			glm::vec3 tangent = glm::normalize(tangentVelocity);
			float tangentSpeed = glm::dot(tangent, tangentVelocity);

			if (tangentSpeed > -TANGENT_STOP_VELOCITY)
			{
				// std::cout << "tangent in!!\n";

				isStop = false;


				// std::cout << "releativeVelocity: " << relativeVelocity.x << " " << relativeVelocity.y << " " << relativeVelocity.z << "\n";
				// std::cout << "tangentVelocity: " << tangentVelocity.x << " " << tangentVelocity.y << " " << tangentVelocity.z << "\n";
				float oldTangentImpulse = manifoldPoint.tangentImpulse;
				float newTangentImpulse = tangentSpeed / pointCount;

				float inverseMasses = (velocityConstraint.invMassA + velocityConstraint.invMassB);
		
				float tangentEffectiveMassA =
					glm::dot(glm::cross(tangent, rA), velocityConstraint.invIA * glm::cross(tangent, rA));
				float tangentEffectiveMassB =
					glm::dot(glm::cross(tangent, rB), velocityConstraint.invIB * glm::cross(tangent, rB));
				newTangentImpulse = newTangentImpulse / (inverseMasses + tangentEffectiveMassA + tangentEffectiveMassB);
				newTangentImpulse += oldTangentImpulse;

				float maxFriction = velocityConstraint.friction * manifoldPoint.normalImpulse;
				// std::cout << "maxFriction: " << maxFriction << "\n";
				// std::cout << "newTangentImpulse: " << newTangentImpulse << "\n";
				newTangentImpulse = glm::clamp(newTangentImpulse, -maxFriction, maxFriction);

				manifoldPoint.tangentImpulse = newTangentImpulse;

				float appliedTangentImpulse = newTangentImpulse - oldTangentImpulse;
				
				// std::cout << "\n\ntangent: " << tangent.x << " " << tangent.y << " " << tangent.z << "\n";
				// std::cout << "appliedTangnentImpulse: " << appliedTangentImpulse << "\n";

				if (appliedTangentImpulse > 0.0f)
				{
					linearVelocityBufferA += velocityConstraint.invMassA * appliedTangentImpulse * tangent;
					linearVelocityBufferB -= velocityConstraint.invMassB * appliedTangentImpulse * tangent;
					angularVelocityBufferA +=
						velocityConstraint.invIA * glm::cross(rA, appliedTangentImpulse * tangent);
					angularVelocityBufferB -=
						velocityConstraint.invIB * glm::cross(rB, appliedTangentImpulse * tangent);
					
					// std::cout << "\n\nAAAA\n";
					// std::cout << "before: " << glm::cross(rA, appliedTangentImpulse * tangent).x << " "
					// 		<< glm::cross(rA, appliedTangentImpulse * tangent).y << " "
					// 		<< glm::cross(rA, appliedTangentImpulse * tangent).z << "\n";
							
					// std::cout << "after: " << (velocityConstraint.invIA * glm::cross(rA, appliedTangentImpulse * tangent)).x << " "
					// 		<< (velocityConstraint.invIA * glm::cross(rA, appliedTangentImpulse * tangent)).y << " "
					// 		<< (velocityConstraint.invIA * glm::cross(rA, appliedTangentImpulse * tangent)).z << "\n";
					
					// std::cout << "\n\nBBB\n";
					// std::cout << "before: " << glm::cross(rB, appliedTangentImpulse * tangent).x << " "
					// 		<< glm::cross(rB, appliedTangentImpulse * tangent).y << " "
					// 		<< glm::cross(rB, appliedTangentImpulse * tangent).z << "\n";
							
					// std::cout << "after: " << (velocityConstraint.invIB * glm::cross(rB, appliedTangentImpulse * tangent)).x << " "
					// 		<< (velocityConstraint.invIB * glm::cross(rB, appliedTangentImpulse * tangent)).y << " "
					// 		<< (velocityConstraint.invIB * glm::cross(rB, appliedTangentImpulse * tangent)).z << "\n";
				}
			}
			else if (-tangentSpeed < TANGENT_SLEEP_VELOCITY)
			{
				// std::cout << "\n\ntangent stop!!\n";
				isStop = false;
			}
			else
			{
				// std::cout << "\n\ntangent stop!!\n";
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

	for (int32_t i = 0; i < m_bodyCount; ++i)
	{
		m_velocities[i].linearVelocity += m_velocities[i].linearVelocityBuffer;
		m_velocities[i].angularVelocity += m_velocities[i].angularVelocityBuffer;
		m_velocities[i].linearVelocityBuffer = glm::vec3(0.0f);
		m_velocities[i].angularVelocityBuffer = glm::vec3(0.0f);
		
		// std::cout << "\n\nafter angular velocity!!!\n";
		// std::cout << "angular velocity: " << m_velocities[i].angularVelocity.x << " " << m_velocities[i].angularVelocity.y << " " << m_velocities[i].angularVelocity.z << "\n";
	}
}

void ContactSolver::solvePositionConstraints()
{
	const float kSlop = 0.001f; // 허용 관통 오차
	const float alpha = 1.0f;

	for (int i = 0; i < m_contactCount; ++i)
	{
		Contact *contact = m_contacts[i];
		ContactPositionConstraint &positionConstraint = m_positionConstraints[i];

		int32_t pointCount = positionConstraint.pointCount;
		int32_t indexA = positionConstraint.indexA;
		int32_t indexB = positionConstraint.indexB;

		float tmpResolvedSeperation = 0.0f;
		float &resolvedSeperation = positionConstraint.resolvedSeperation;

		glm::mat3 &invIA = positionConstraint.invIA;
		glm::mat3 &invIB = positionConstraint.invIB;

		float sumMass = positionConstraint.invMassA + positionConstraint.invMassB;
		float ratioA = positionConstraint.invMassA / sumMass;
		float ratioB = positionConstraint.invMassB / sumMass;

		glm::vec3 &positionBufferA = m_positions[indexA].positionBuffer;
		glm::vec3 &positionBufferB = m_positions[indexB].positionBuffer;

		for (int32_t j = 0; j < pointCount; j++)
		{

			ManifoldPoint &manifoldPoint = positionConstraint.points[j];

			float seperation = manifoldPoint.seperation - resolvedSeperation;

			if (seperation <= 0.0f)
			{
				continue;
			}

			// 관통 해소된상태면 무시
			if (seperation < kSlop)
			{
				continue;
			}

			// 관통 깊이에 따른 보정량 계산
			float correction = seperation * alpha / pointCount;
			glm::vec3 correctionVector = correction * manifoldPoint.normal;

			tmpResolvedSeperation += correction;

			positionBufferA -= correctionVector * ratioA;
			positionBufferB += correctionVector * ratioB;
		}

		resolvedSeperation += tmpResolvedSeperation;
	}
}

} // namespace ale
