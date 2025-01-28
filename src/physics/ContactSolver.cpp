#include "physics/ContactSolver.h"

namespace ale
{

const float ContactSolver::NORMAL_STOP_VELOCITY = 0.0001f;
const float ContactSolver::TANGENT_STOP_VELOCITY = 0.0001f;
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

		// std::cout << "\n\nbefore velocity!!!\n";
		// std::cout << "linear velocityA: " << linearVelocityA.x << " " << linearVelocityA.y << " " << linearVelocityA.z << "\n";
		// std::cout << "linear velocityB: " << linearVelocityB.x << " " << linearVelocityB.y << " " << linearVelocityB.z << "\n";
		// std::cout << "angular velocityA: " << angularVelocityA.x << " " << angularVelocityA.y << " " << angularVelocityA.z << "\n";
		// std::cout << "angular velocityB: " << angularVelocityB.x << " " << angularVelocityB.y << " " << angularVelocityB.z << "\n";

		glm::vec3 &linearVelocityBufferA = m_velocities[indexA].linearVelocityBuffer;
		glm::vec3 &linearVelocityBufferB = m_velocities[indexB].linearVelocityBuffer;
		glm::vec3 &angularVelocityBufferA = m_velocities[indexA].angularVelocityBuffer;
		glm::vec3 &angularVelocityBufferB = m_velocities[indexB].angularVelocityBuffer;

		bool isStop = true;

		float seperationSum = 0.0f;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			seperationSum += velocityConstraint.points[j].seperation;
		}

		if (seperationSum <= 0.0f)
		{
			continue ;
		}

		for (int32_t j = 0; j < pointCount; ++j)
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

			if (normalSpeed < -NORMAL_STOP_VELOCITY)
			{
				// std::cout << "normal in!!\n";

				float oldNormalImpulse = manifoldPoint.normalImpulse;
				appliedNormalImpulse = -(1.0f + velocityConstraint.restitution) * normalSpeed * (manifoldPoint.seperation / seperationSum);
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

				// if (appliedNormalImpulse > 400.0f || appliedNormalImpulse < 0.0f)
				{
					// std::cout << "normalSpeed: " << normalSpeed << "\n"; 
					// std::cout << "normalImpusle: " << appliedNormalImpulse << "\n";
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

			// 접선 방향 충격량 계산
			glm::vec3 tangentVelocity = relativeVelocity - (normalSpeed * manifoldPoint.normal);
			glm::vec3 tangent = glm::normalize(tangentVelocity);
			float tangentSpeed = glm::dot(tangent, tangentVelocity);

			if (tangentSpeed > TANGENT_STOP_VELOCITY)
			{
				// std::cout << "tangent in!!\n";

				// std::cout << "releativeVelocity: " << relativeVelocity.x << " " << relativeVelocity.y << " " << relativeVelocity.z << "\n";
				// std::cout << "tangentVelocity: " << tangentVelocity.x << " " << tangentVelocity.y << " " << tangentVelocity.z << "\n";
				// std::cout << "tangentSpeed: " << tangentSpeed << "\n";
				float oldTangentImpulse = manifoldPoint.tangentImpulse;
				float newTangentImpulse = tangentSpeed * (manifoldPoint.seperation / seperationSum);

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
				
				// if (appliedTangentImpulse > 100.0f)
				// {
				// 	std::cout << "\n\ntangent: " << tangent.x << " " << tangent.y << " " << tangent.z << "\n";
				// 	std::cout << "appliedTangnentImpulse: " << appliedTangentImpulse << "\n";
				// }

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

			// glm::vec3 relativeAngularVelocity = angularVelocityB - angularVelocityA;

			// if (glm::length(relativeAngularVelocity) > 0.0001f)
			// {

			// 	// 회전 축 벡터 (법선 방향에 평행)
			// 	glm::vec3 rotationalAxis = glm::normalize(relativeAngularVelocity); // 상대 각속도의 방향

			// 	// 유효 관성 계산
			// 	float rotationalEffectiveMassA = glm::dot(rotationalAxis, velocityConstraint.invIA * rotationalAxis);
			// 	float rotationalEffectiveMassB = glm::dot(rotationalAxis, velocityConstraint.invIB * rotationalAxis);
			// 	float rotationalEffectiveMass = 1.0f / (rotationalEffectiveMassA + rotationalEffectiveMassB);

			// 	// 회전 마찰 토크 계산
			// 	glm::vec3 rotationalFrictionTorque = -rotationalEffectiveMass * relativeAngularVelocity * (manifoldPoint.seperation / seperationSum);

			// 	// 최대 회전 마찰 제한
			// 	float maxRotationalFriction = velocityConstraint.friction * manifoldPoint.normalImpulse;
			// 	float torqueMagnitude = glm::length(rotationalFrictionTorque);
			// 	if (torqueMagnitude > maxRotationalFriction)
			// 	{
			// 		rotationalFrictionTorque *= (maxRotationalFriction / torqueMagnitude);
			// 	}

			// 	// 각속도 업데이트
			// 	angularVelocityBufferA -= velocityConstraint.invIA * rotationalFrictionTorque;
			// 	angularVelocityBufferB += velocityConstraint.invIB * rotationalFrictionTorque;
			// }

		}
		linearVelocityA += linearVelocityBufferA;
		linearVelocityB += linearVelocityBufferB;
		angularVelocityA += angularVelocityBufferA;
		angularVelocityB += angularVelocityBufferB;
		linearVelocityBufferA = glm::vec3(0.0f);
		linearVelocityBufferB = glm::vec3(0.0f);
		angularVelocityBufferA = glm::vec3(0.0f);
		angularVelocityBufferB = glm::vec3(0.0f);
	}

	for (int32_t i = 0; i < m_bodyCount; ++i)
	{
		
		// std::cout << "\n\nafter velocity!!!\n";
		// std::cout << "linear velocity: " << m_velocities[i].linearVelocity.x << " " << m_velocities[i].linearVelocity.y << " " << m_velocities[i].linearVelocity.z << "\n";
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

			glm::vec3 movedPointA = manifoldPoint.pointA + positionBufferA;
			glm::vec3 movedPointB = manifoldPoint.pointB + positionBufferB;

			float seperation = glm::dot(manifoldPoint.normal, movedPointA - movedPointB);

			// 관통 해소된상태면 무시
			if (seperation < kSlop)
			{
				continue;
			}

			// 관통 깊이에 따른 보정량 계산
			float correction = seperation * alpha / pointCount;
			glm::vec3 correctionVector = correction * manifoldPoint.normal;

			positionBufferA -= correctionVector * ratioA;
			positionBufferB += correctionVector * ratioB;
		}
	}
}

void ContactSolver::checkSleepContact()
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
		glm::vec3 upVector = glm::vec3(0.0f, 1.0f, 0.0f);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			ManifoldPoint &manifoldPoint = velocityConstraint.points[j];
			
			glm::vec3 relativeVelocity = linearVelocityA - linearVelocityB;
			
			if (glm::length(relativeVelocity) > 1.0f)
			{
				m_positions[indexA].isNormalStop = false;
				m_positions[indexB].isNormalStop = false;
			}

			glm::vec3 relativeAngularVelocity = angularVelocityA - angularVelocityB;
			if (glm::length(relativeAngularVelocity) > 1.0f)
			{
				m_positions[indexA].isTangentStop = false;
				m_positions[indexB].isTangentStop = false;
			}

			float normalDotUpVector = glm::dot(manifoldPoint.normal, upVector);
			if (normalDotUpVector < -0.95f)
			{
				// std::cout << "!!!!!!!!!!!!!!!!!!seperation: " << seperationSum << "\n";
				m_positions[indexA].isNormal = true;
			}
			if (normalDotUpVector > 0.95f)
			{
				m_positions[indexB].isNormal = true;
			}
		}

	}
}


} // namespace ale
