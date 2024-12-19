#include "physics/ContactSolver.h"

namespace ale
{

ContactSolver::ContactSolver(float duration, std::vector<Contact *> &contacts, std::vector<Position> &positions,
							 std::vector<Velocity> &velocities)
	: m_duration(duration), m_positions(positions), m_velocities(velocities), m_contacts(contacts),
	  m_stopVelocity(duration)
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

		glm::vec3 gravity = Rigidbody::gravity;
		bool isStopContact = true;

		if (bodyA->getType() == BodyType::e_dynamic)
		{
			float gravityVelocity =
				glm::dot(((gravity / bodyA->getInverseMass()) * m_duration), manifold.points[0].normal);
			float normalVelocity =
				glm::dot(m_velocities[bodyA->getIslandIndex()].linearVelocity, manifold.points[0].normal);
			// std::cout << "A gravityVelocity: " << gravityVelocity << "\n";
			// std::cout << "A normalVelocity: " << normalVelocity << "\n";
			isStopContact = (m_stopVelocity >= std::abs(normalVelocity - gravityVelocity));
		}

		if (bodyB->getType() == BodyType::e_dynamic)
		{
			float gravityVelocity =
				glm::dot(((gravity / bodyB->getInverseMass()) * m_duration), -manifold.points[0].normal);
			float normalVelocity =
				glm::dot(m_velocities[bodyB->getIslandIndex()].linearVelocity, -manifold.points[0].normal);
			// std::cout << "B gravityVelocity: " << gravityVelocity << "\n";
			// std::cout << "B normalVelocity: " << normalVelocity << "\n";
			isStopContact = (m_stopVelocity >= std::abs(normalVelocity - gravityVelocity));
		}

		if (isStopContact)
			std::cout << "STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
		else
			std::cout << "notStop\n";
			
		// 속도 제약 설정
		ContactVelocityConstraint velocityConstraint;
		velocityConstraint.friction = contact->getFriction();
		velocityConstraint.restitution = contact->getRestitution();
		velocityConstraint.worldCenterA = bodyA->getTransform().toMatrix() * glm::vec4(shapeA->localCenter, 1.0f);
		velocityConstraint.worldCenterB = bodyB->getTransform().toMatrix() * glm::vec4(shapeB->localCenter, 1.0f);
		velocityConstraint.indexA = bodyA->getIslandIndex();
		velocityConstraint.indexB = bodyB->getIslandIndex();
		velocityConstraint.invMassA = bodyA->getInverseMass();
		velocityConstraint.invMassB = bodyB->getInverseMass();
		velocityConstraint.invIA = bodyA->getInverseInertiaTensorWorld();
		velocityConstraint.invIB = bodyB->getInverseInertiaTensorWorld();
		velocityConstraint.pointCount = manifold.points.size();
		velocityConstraint.points = manifold.points;
		velocityConstraint.isStopContact = isStopContact;

		// 위치 제약 설정
		ContactPositionConstraint positionConstraint;
		positionConstraint.worldCenterA = bodyA->getTransform().toMatrix() * glm::vec4(shapeA->localCenter, 1.0f);
		positionConstraint.worldCenterB = bodyB->getTransform().toMatrix() * glm::vec4(shapeB->localCenter, 1.0f);
		positionConstraint.indexA = bodyA->getIslandIndex();
		positionConstraint.indexB = bodyB->getIslandIndex();
		positionConstraint.invMassA = bodyA->getInverseMass();
		positionConstraint.invMassB = bodyB->getInverseMass();
		positionConstraint.pointCount = manifold.points.size();
		positionConstraint.points = manifold.points;
		positionConstraint.isStopContact = isStopContact;

		m_velocityConstraints.push_back(velocityConstraint);
		m_positionConstraints.push_back(positionConstraint);
	}
}

void ContactSolver::solveVelocityConstraints(const int32_t velocityIteration)
{
	int32_t length = m_contacts.size();
	for (int32_t i = 0; i < length; i++)
	{
		Contact *contact = m_contacts[i];
		ContactVelocityConstraint &velocityConstraint = m_velocityConstraints[i];
		int32_t pointCount = velocityConstraint.pointCount;
		int32_t indexA = velocityConstraint.indexA;
		int32_t indexB = velocityConstraint.indexB;
		std::cout << "before velocityA: " << m_velocities[indexA].linearVelocity.x << " " << m_velocities[indexA].linearVelocity.y << " " << m_velocities[indexA].linearVelocity.z << "\n";
		std::cout << "before velocityB: " << m_velocities[indexB].linearVelocity.x << " " << m_velocities[indexB].linearVelocity.y << " " << m_velocities[indexB].linearVelocity.z << "\n";
		std::cout << "before angularVelocityA: " << m_velocities[indexA].angularVelocity.x << " " << m_velocities[indexA].angularVelocity.y << " " << m_velocities[indexA].angularVelocity.z << "\n";
		std::cout << "before angularVelocityB: " << m_velocities[indexB].angularVelocity.x << " " << m_velocities[indexB].angularVelocity.y << " " << m_velocities[indexB].angularVelocity.z << "\n";

		for (int32_t j = 0; j < pointCount; j++)
		{
			ManifoldPoint &manifoldPoint = velocityConstraint.points[j];
			glm::vec3 rA = manifoldPoint.pointA - velocityConstraint.worldCenterA; // bodyA의 충돌 지점까지의 벡터
			glm::vec3 rB = manifoldPoint.pointB - velocityConstraint.worldCenterB; // bodyB의 충돌 지점까지의 벡터
			// std::cout << "rA: " << rA.x << " " << rA.y << " " << rA.z << "\n";
			// std::cout << "rB: " << rB.x << " " << rB.y << " " << rB.z << "\n";
			// 상대 속도 계산
			glm::vec3 relativeVelocity =
				(m_velocities[indexB].linearVelocity + glm::cross(m_velocities[indexB].angularVelocity, rB)) -
				(m_velocities[indexA].linearVelocity + glm::cross(m_velocities[indexA].angularVelocity, rA));
			std::cout << "relativeVelocity: " << relativeVelocity.x << " " << relativeVelocity.y << " " << relativeVelocity.z << "\n";
			std::cout << "normalVector: " << manifoldPoint.normal.x  << " " << manifoldPoint.normal.y << " " << manifoldPoint.normal.z << "\n";
			// 법선 방향 속도
			float normalVelocity = glm::dot(relativeVelocity, manifoldPoint.normal);
			std::cout << "normalVelocity: " << normalVelocity << "\n";
			// if (normalVelocity > -m_stopVelocity)
			// {
			// 	continue;
			// }

			// 법선 방향 충격량 계산
			float normalImpulse = -(1.0f + velocityConstraint.restitution) * normalVelocity;
			float inverseMasses = (velocityConstraint.invMassA + velocityConstraint.invMassB);
			float normalEffectiveMassA = glm::dot(glm::cross(manifoldPoint.normal, rA), velocityConstraint.invIA * glm::cross(manifoldPoint.normal, rA));
			float normalEffectiveMassB = glm::dot(glm::cross(manifoldPoint.normal, rB), velocityConstraint.invIB * glm::cross(manifoldPoint.normal, rB));
			std::cout << "normalEffectiveMassA: "<< normalEffectiveMassA << "\n";
			std::cout << "normalEffectiveMassB: "<< normalEffectiveMassB << "\n";

			normalImpulse = normalImpulse / (inverseMasses + normalEffectiveMassA + normalEffectiveMassB);
			std::cout << "normalImpulse: " << normalImpulse << "\n"; 

			glm::vec3 tangent(0.0f);
			glm::vec3 tangentVelocity = relativeVelocity - (normalVelocity * manifoldPoint.normal);
			if (glm::length2(tangentVelocity) != 0)
			{
				tangent = glm::normalize(tangentVelocity);
			}

			float tangentImpulse = -glm::dot(relativeVelocity, tangent);
			float tangentEffectiveMassA = glm::dot(glm::cross(tangent, rA), velocityConstraint.invIA * glm::cross(tangent, rA)); 
			float tangentEffectiveMassB = glm::dot(glm::cross(tangent, rB), velocityConstraint.invIB * glm::cross(tangent, rB)); 
			float maxFriction = velocityConstraint.friction * normalImpulse;
			std::cout << "inverseMasses: " << inverseMasses << "\n";
			std::cout << "tangentEffectiveMassA: " << tangentEffectiveMassA << "\n";
			std::cout << "tangentEffectiveMassB: " << tangentEffectiveMassB << "\n";
			tangentImpulse = tangentImpulse / (inverseMasses + tangentEffectiveMassA + tangentEffectiveMassB);
			tangentImpulse = glm::clamp(tangentImpulse, -maxFriction, maxFriction);
			std::cout << "tangentImpulse: " << tangentImpulse << "\n"; 

			if (velocityConstraint.isStopContact)
			{
				std::cout << "Stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
				m_velocities[indexA].linearVelocity -= glm::dot(m_velocities[indexA].linearVelocity, manifoldPoint.normal) * manifoldPoint.normal;
				m_velocities[indexB].linearVelocity -= glm::dot(m_velocities[indexB].linearVelocity, manifoldPoint.normal) * manifoldPoint.normal;
			}
			else
			{
				m_velocities[indexA].linearVelocity -= velocityConstraint.invMassA * normalImpulse * manifoldPoint.normal * 1.0f;
				m_velocities[indexB].linearVelocity += velocityConstraint.invMassB * normalImpulse * manifoldPoint.normal * 1.0f;
				m_velocities[indexA].angularVelocity -= velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal);
				m_velocities[indexB].angularVelocity += velocityConstraint.invIB * glm::cross(rB, normalImpulse * manifoldPoint.normal);	
			}
		}
	
		std::cout << "after velocityA: " << m_velocities[indexA].linearVelocity.x << " " << m_velocities[indexA].linearVelocity.y << " " << m_velocities[indexA].linearVelocity.z << "\n";
		std::cout << "after velocityB: " << m_velocities[indexB].linearVelocity.x << " " << m_velocities[indexB].linearVelocity.y << " " << m_velocities[indexB].linearVelocity.z << "\n";		
		std::cout << "after angularVelocityA: " << m_velocities[indexA].angularVelocity.x << " " << m_velocities[indexA].angularVelocity.y << " " << m_velocities[indexA].angularVelocity.z << "\n";
		std::cout << "after angularVelocityB: " << m_velocities[indexB].angularVelocity.x << " " << m_velocities[indexB].angularVelocity.y << " " << m_velocities[indexB].angularVelocity.z << "\n";
	}
}

bool ContactSolver::solvePositionConstraints(const int32_t positionIteration)
{
	const float kSlop = 0.001f;	// 허용 관통 오차
	bool solved = true;

	int32_t length = m_contacts.size();
	for (int i = 0; i < length; ++i)
	{
		Contact *contact = m_contacts[i];
		ContactPositionConstraint &positionConstraint = m_positionConstraints[i];
		int32_t pointCount = positionConstraint.pointCount;
		int32_t indexA = positionConstraint.indexA;
		int32_t indexB = positionConstraint.indexB;
		bool isStopContact = positionConstraint.isStopContact;
		glm::vec3 positionA(0.0f);
		glm::vec3 positionB(0.0f);
		for (int32_t j = 0; j < pointCount; j++)
		{

			ManifoldPoint &manifoldPoint = positionConstraint.points[j];
			std::cout << "seperation: " << manifoldPoint.seperation << "\n"; 

			glm::vec3 rA = manifoldPoint.pointA - positionConstraint.worldCenterA; // bodyA의 충돌 지점까지의 벡터
			glm::vec3 rB = manifoldPoint.pointB - positionConstraint.worldCenterB; // bodyB의 충돌 지점까지의 벡터
			// 상대 속도 계산
			glm::vec3 relativeVelocity =
				(m_velocities[indexB].linearVelocity + glm::cross(m_velocities[indexB].angularVelocity, rB)) -
				(m_velocities[indexA].linearVelocity + glm::cross(m_velocities[indexA].angularVelocity, rA));
			// 관통 해소된상태면 무시
			if (glm::dot(manifoldPoint.pointA - manifoldPoint.pointB, manifoldPoint.normal) < kSlop)
			{
				std::cout << "pointA : " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " " << manifoldPoint.pointA.z << "\n";
				std::cout << "pointB : " << manifoldPoint.pointB.x << " " << manifoldPoint.pointB.y << " " << manifoldPoint.pointB.z << "\n";
				std::cout << "normal : " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " " << manifoldPoint.normal.z << "\n";

				std::cout << "Siu!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
				continue;
			}
			// 관통 깊이에 따른 보정량 계산

			float correction = manifoldPoint.seperation;
			glm::vec3 correctionVector = correction * glm::normalize(manifoldPoint.pointA - manifoldPoint.pointB);

			std::cout << "correction: " << correction << "\n";
			std::cout << "normal : " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " " << manifoldPoint.normal.z << "\n";
			std::cout << "correctionVector : " << correctionVector.x << " " << correctionVector.y << " " << correctionVector.z << "\n";

			// 위치 보정
			positionA -= correctionVector * positionConstraint.invMassA /
						 (positionConstraint.invMassA + positionConstraint.invMassB);
			positionB += correctionVector * positionConstraint.invMassB /
						 (positionConstraint.invMassA + positionConstraint.invMassB);
			std::cout << "pos dA: " << positionA.x << " " << positionA.y << " " << positionA.z << "\n";
			std::cout << "pos dB: " << positionB.x << " " << positionB.y << " " << positionB.z << "\n";
			
			manifoldPoint.pointA += positionA;
			manifoldPoint.pointB += positionB;
			if (glm::dot(manifoldPoint.pointA - manifoldPoint.pointB, manifoldPoint.normal) > kSlop)
			{
				solved = false;
			}
		}

		m_positions[indexA].position += positionA;
		m_positions[indexB].position += positionB;
	}
	return solved;
}

} // namespace ale
