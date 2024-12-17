#include "physics/ContactSolver.h"

namespace ale
{

ContactSolver::ContactSolver(float duration, std::vector<Contact *> &contacts, std::vector<Position> &positions,
							 std::vector<Velocity> &velocities)
	: m_duration(duration), m_positions(positions), m_velocities(velocities), m_contacts(contacts),
	  m_stopVelocity(duration * (3.0f / 5.0f))
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

		glm::vec3 gravity(0.0f, -1.0f, 0.0f);
		bool isStopContact = true;

		if (bodyA->getType() == BodyType::e_dynamic)
		{
			float gravityVelocity =
				glm::dot((gravity * bodyA->getInverseMass() * m_duration), manifold.points[0].normal);
			float normalVelocity =
				glm::dot(m_velocities[bodyA->getIslandIndex()].linearVelocity, manifold.points[0].normal);
			isStopContact = (m_stopVelocity >= (normalVelocity - gravityVelocity));
		}

		if (bodyB->getType() == BodyType::e_dynamic)
		{
			float gravityVelocity =
				glm::dot((gravity * bodyB->getInverseMass() * m_duration), -manifold.points[0].normal);
			float normalVelocity =
				glm::dot(m_velocities[bodyB->getIslandIndex()].linearVelocity, -manifold.points[0].normal);
			isStopContact = (m_stopVelocity >= (normalVelocity - gravityVelocity));
		}

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

		glm::vec3 linearVelocityA(0.0f);
		glm::vec3 linearVelocityB(0.0f);
		glm::vec3 angularVelocityA(0.0f);
		glm::vec3 angularVelocityB(0.0f);

		for (int32_t j = 0; j < pointCount; j++)
		{
			ManifoldPoint &manifoldPoint = velocityConstraint.points[j];
			// 상대 속도 계산
			glm::vec3 rA = manifoldPoint.pointA - velocityConstraint.worldCenterA; // bodyA의 충돌 지점까지의 벡터
			glm::vec3 rB = manifoldPoint.pointB - velocityConstraint.worldCenterB; // bodyB의 충돌 지점까지의 벡터
			glm::vec3 relativeVelocity =
				(m_velocities[indexB].linearVelocity + glm::cross(m_velocities[indexB].angularVelocity, rB)) -
				(m_velocities[indexA].linearVelocity + glm::cross(m_velocities[indexA].angularVelocity, rA));

			// 법선 방향 속도
			float normalVelocity = glm::dot(relativeVelocity, manifoldPoint.normal);

			if (normalVelocity < m_stopVelocity)
			{
				continue;
			}

			// 법선 방향 충격량 계산
			float normalImpulse = -(1.0f + velocityConstraint.restitution) * normalVelocity;
			normalImpulse = normalImpulse / (velocityConstraint.invMassA + velocityConstraint.invMassB);
			float alpha = 1.0f / float(velocityIteration); // 반복 횟수에 따른 비율
			normalImpulse *= alpha;

			// 충격량 클램핑 (누적된 충격량 포함)
			float prevNormalImpulse = manifoldPoint.normalImpulse;
			manifoldPoint.normalImpulse = glm::max(prevNormalImpulse + normalImpulse, 0.0f);
			normalImpulse = manifoldPoint.normalImpulse - prevNormalImpulse;

			// 접선 방향 마찰 계산
			glm::vec3 tangent = relativeVelocity - (normalVelocity * manifoldPoint.normal);
			if (glm::length(tangent) > 0.0f)
			{
				tangent = glm::normalize(tangent);
			}
			float tangentImpulse = -glm::dot(relativeVelocity, tangent);
			tangentImpulse = tangentImpulse / (velocityConstraint.invMassA + velocityConstraint.invMassB);

			// 마찰 충격량 클램핑
			float maxFriction = velocityConstraint.friction * manifoldPoint.normalImpulse;
			float prevTangentImpulse = manifoldPoint.tangentImpulse;
			manifoldPoint.tangentImpulse = glm::clamp(prevTangentImpulse + tangentImpulse, -maxFriction, maxFriction);
			tangentImpulse = manifoldPoint.tangentImpulse - prevTangentImpulse;

			// 총 충격량
			glm::vec3 totalImpulse = (normalImpulse * manifoldPoint.normal) + (tangentImpulse * tangent);
			// 속도 업데이트
			if (velocityConstraint.isStopContact)
			{
				linearVelocityA =
					linearVelocityA -
					glm::dot(m_velocities[indexA].linearVelocity, manifoldPoint.normal) * manifoldPoint.normal;
				linearVelocityB =
					linearVelocityB -
					glm::dot(m_velocities[indexB].linearVelocity, -manifoldPoint.normal) * -manifoldPoint.normal;
				manifoldPoint.normalImpulse = 0.0f;
			}
			else
			{
				linearVelocityA = linearVelocityA - totalImpulse * velocityConstraint.invMassA;
				linearVelocityB = linearVelocityB + totalImpulse * velocityConstraint.invMassB;
			}

			// 각속도 업데이트
			glm::vec3 angularImpulseA = glm::cross(rA, totalImpulse) * velocityConstraint.invIA;
			glm::vec3 angularImpulseB = glm::cross(rB, -totalImpulse) * velocityConstraint.invIB;
			angularVelocityA = angularVelocityA - angularImpulseA;
			angularVelocityB = angularVelocityB + angularImpulseB;
		}
		m_velocities[indexA].linearVelocity += linearVelocityA;
		m_velocities[indexA].angularVelocity += angularVelocityA;
		m_velocities[indexB].linearVelocity += linearVelocityB;
		m_velocities[indexB].angularVelocity += angularVelocityB;
	}
}

bool ContactSolver::solvePositionConstraints(const int32_t positionIteration)
{
	const float kSlop = 0.01f;					  // 허용 관통 오차
	const float beta = positionIteration * 0.01f; // 보정 계수

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

			// 관통 해소된상태면 무시
			if (glm::dot(manifoldPoint.pointA - manifoldPoint.pointB, manifoldPoint.normal) < kSlop)
			{
				continue;
			}
			// 관통 깊이에 따른 보정량 계산
			float correction = beta * (manifoldPoint.seperation - kSlop);
			if (positionConstraint.isStopContact)
			{
				std::cout << "Stop Contact!!\n";
				correction = manifoldPoint.seperation;
			}
			glm::vec3 correctionVector = correction * manifoldPoint.normal;

			// 위치 보정
			positionA -= correctionVector * positionConstraint.invMassA /
						 (positionConstraint.invMassA + positionConstraint.invMassB);
			positionB += correctionVector * positionConstraint.invMassB /
						 (positionConstraint.invMassA + positionConstraint.invMassB);
			std::cout << "pos dA: " << positionA.x << " " << positionA.y << " " << positionA.z << "\n";
			std::cout << "pos dB: " << positionB.x << " " << positionB.y << " " << positionB.z << "\n";
			
			std::cout << "Before pointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " " << manifoldPoint.pointA.z << "\n";
			std::cout << "Before pointB: " << manifoldPoint.pointB.x << " " << manifoldPoint.pointB.y << " " << manifoldPoint.pointB.z << "\n";
			manifoldPoint.pointA += positionA;
			manifoldPoint.pointB += positionB;
			std::cout << "Before pointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " " << manifoldPoint.pointA.z << "\n";
			std::cout << "Before pointB: " << manifoldPoint.pointB.x << " " << manifoldPoint.pointB.y << " " << manifoldPoint.pointB.z << "\n";
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
