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

		// if (bodyA->getType() == BodyType::e_dynamic)
		// {
		// 	float gravityVelocity = glm::dot((gravity * m_duration), manifold.points[0].normal);
		// 	float normalVelocity =
		// 		glm::dot(m_velocities[bodyA->getIslandIndex()].linearVelocity, manifold.points[0].normal);
		// 	// std::cout << "A gravityVelocity: " << gravityVelocity << "\n";
		// 	// std::cout << "A normalVelocity: " << normalVelocity << "\n";
		// 	isStopContact = (m_stopVelocity >= std::abs(normalVelocity - gravityVelocity));
		// }

		// if (bodyB->getType() == BodyType::e_dynamic)
		// {
		// 	float gravityVelocity = glm::dot((gravity * m_duration), -manifold.points[0].normal);
		// 	float normalVelocity =
		// 		glm::dot(m_velocities[bodyB->getIslandIndex()].linearVelocity, -manifold.points[0].normal);
		// 	// std::cout << "B gravityVelocity: " << gravityVelocity << "\n";
		// 	// std::cout << "B normalVelocity: " << normalVelocity << "\n";
		// 	isStopContact = (m_stopVelocity >= std::abs(normalVelocity - gravityVelocity));
		// }

		// if (isStopContact)
		// 	std::cout << "STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
		// else
		// 	std::cout << "notStop\n";

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

		// std::cout << "worldCenterA: " << velocityConstraint.worldCenterA.x << " " <<
		// velocityConstraint.worldCenterA.y << " " << velocityConstraint.worldCenterA.z << "\n";

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
		// std::cout << "\n\nbodyA: " << contact->getFixtureA()->getBody()->getBodyId() << "\n";
		// std::cout << "bodyB: " << contact->getFixtureB()->getBody()->getBodyId() << "\n";
		// std::cout << "before velocityA: " << m_velocities[indexA].linearVelocity.x << " "
		// 		  << m_velocities[indexA].linearVelocity.y << " " << m_velocities[indexA].linearVelocity.z << "\n";
		// std::cout << "before velocityB: " << m_velocities[indexB].linearVelocity.x << " "
		// 		  << m_velocities[indexB].linearVelocity.y << " " << m_velocities[indexB].linearVelocity.z << "\n";
		// std::cout << "before angularVelocityA: " << m_velocities[indexA].angularVelocity.x << " "
		// 		  << m_velocities[indexA].angularVelocity.y << " " << m_velocities[indexA].angularVelocity.z << "\n";
		// std::cout << "before angularVelocityB: " << m_velocities[indexB].angularVelocity.x << " "
		// 		  << m_velocities[indexB].angularVelocity.y << " " << m_velocities[indexB].angularVelocity.z << "\n";

		float seperationSum = 0;
		for (int32_t j = 0; j < pointCount; j++)
		{
			seperationSum += velocityConstraint.points[j].seperation;
		}
		if (seperationSum < 1e-8)
		{
			seperationSum = 1.0f;
		}

		glm::vec3 initLinearVelocityA = m_velocities[indexA].linearVelocity;
		glm::vec3 initLinearVelocityB = m_velocities[indexB].linearVelocity;
		glm::vec3 initAngularVelocityA = m_velocities[indexA].angularVelocity;
		glm::vec3 initAngularVelocityB = m_velocities[indexB].angularVelocity;
		glm::vec3 finalLinearVelocityA(0.0f);
		glm::vec3 finalLinearVelocityB(0.0f);
		glm::vec3 finalAngularVelocityA(0.0f);
		glm::vec3 finalAngularVelocityB(0.0f);

		for (int32_t j = 0; j < pointCount; j++)
		{
			ManifoldPoint &manifoldPoint = velocityConstraint.points[j];
			// std::cout << "pointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " "
			// 		  << manifoldPoint.pointA.z << "\n";
			// std::cout << "pointB: " << manifoldPoint.pointB.x << " " << manifoldPoint.pointB.y << " "
			// 		  << manifoldPoint.pointB.z << "\n";
			glm::vec3 rA = manifoldPoint.pointA - velocityConstraint.worldCenterA; // bodyA의 충돌 지점까지의 벡터
			glm::vec3 rB = manifoldPoint.pointB - velocityConstraint.worldCenterB; // bodyB의 충돌 지점까지의 벡터
			// std::cout << "RARARA pointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " " << manifoldPoint.pointA.z << "\n"; 
			// std::cout << "worldCenterA: " << velocityConstraint.worldCenterA.x << " " << velocityConstraint.worldCenterA.y << " " << velocityConstraint.worldCenterA.z << "\n"; 
			// std::cout << "worldCenterB: " << velocityConstraint.worldCenterB.x << " " << velocityConstraint.worldCenterB.y << " " << velocityConstraint.worldCenterB.z << "\n"; 
			// std::cout << "rB: " << rB.x << " " << rB.y << " " << rB.z << "\n";

			// 상대 속도 계산
			// A에서 본 B의 상대속도
			glm::vec3 velocityA = initLinearVelocityA + glm::cross(initAngularVelocityA, rA);
			glm::vec3 velocityB = initLinearVelocityB + glm::cross(initAngularVelocityB, rB);
			// std::cout << "velocityA : " << glm::length(velocityA) << "\n";
			// std::cout << "velocityB : " << glm::length(velocityB) << "\n";

			glm::vec3 relativeVelocity = velocityB - velocityA;

			// if (glm::length2(velocityA) < 1.0f && glm::length2(velocityB) < 1.0f)
			// {
			// 	float damping = 0.999f;
			// 	m_velocities[indexA].linearVelocity *= damping;
			// 	m_velocities[indexB].linearVelocity *= damping;
			// 	m_velocities[indexA].angularVelocity *= damping;
			// 	m_velocities[indexB].angularVelocity *= damping;
			// 	return;
			// }

			// std::cout << "relativeVelocity: " << relativeVelocity.x << " " << relativeVelocity.y << " "
			// 		  << relativeVelocity.z << "\n";
			// std::cout << "normalVector: " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " "
			// 		  << manifoldPoint.normal.z << "\n";

			// 법선 방향 속도
			float normalSpeed = glm::dot(relativeVelocity, manifoldPoint.normal); // 음수
			// std::cout << "normalSpeed: " << normalSpeed << "\n";

			// 법선 방향 충격량 계산
			float normalImpulse =
				-(1.0f + velocityConstraint.restitution) * normalSpeed * (manifoldPoint.seperation / seperationSum);
			// std::cout << "before normalImpulse: " << normalImpulse << "\n";
			float inverseMasses = (velocityConstraint.invMassA + velocityConstraint.invMassB);
			float normalEffectiveMassA = glm::dot(glm::cross(manifoldPoint.normal, rA),
												  velocityConstraint.invIA * glm::cross(manifoldPoint.normal, rA));
			float normalEffectiveMassB = glm::dot(glm::cross(manifoldPoint.normal, rB),
												  velocityConstraint.invIB * glm::cross(manifoldPoint.normal, rB));
			// std::cout << "rA : " << rA.x << " " << rA.y << " " << rA.z << "\n";
			// std::cout << "rB : " << rB.x << " " << rB.y << " " << rB.z << "\n";
			// std::cout << "inverseMasses: " << inverseMasses << "\n";
			// std::cout << "normalEffectiveMassA: " << normalEffectiveMassA << "\n";
			// std::cout << "normalEffectiveMassB: " << normalEffectiveMassB << "\n";

			normalImpulse = normalImpulse / (inverseMasses + normalEffectiveMassA + normalEffectiveMassB);
			float normalImpulseForFriction =  normalImpulse;
			if (normalSpeed > -m_stopVelocity)
			{
				normalImpulse = 0.0f;
			}
			// std::cout << "normalImpulse: " << normalImpulse << "\n";

			glm::vec3 tangent(0.0f);
			glm::vec3 tangentVelocity = relativeVelocity - (normalSpeed * manifoldPoint.normal);
			float tangentSpeed = glm::length(tangentVelocity);
			float tangentImpulse;
			// std::cout << "tangentVelocity: " << tangentVelocity.x << " " << tangentVelocity.y << " "
			// 		  << tangentVelocity.z << "\n";
			// std::cout << "tangentSpeed: " << tangentSpeed << "\n";
			if (tangentSpeed > m_stopVelocity)
			{
				tangent = glm::normalize(tangentVelocity);

				tangentImpulse = -glm::dot(relativeVelocity, tangent);
				// std::cout << "tangent: " << tangent.x << " " << tangent.y << " " << tangent.z << "\n";

				float tangentEffectiveMassA =
					glm::dot(glm::cross(tangent, rA), velocityConstraint.invIA * glm::cross(tangent, rA));
				float tangentEffectiveMassB =
					glm::dot(glm::cross(tangent, rB), velocityConstraint.invIB * glm::cross(tangent, rB));
				float maxFriction = velocityConstraint.friction * normalImpulseForFriction;
				tangentImpulse = tangentImpulse / (inverseMasses + tangentEffectiveMassA + tangentEffectiveMassB);
				// std::cout << "before clamp tangentImpulse: " << tangentImpulse << "\n";
				tangentImpulse = glm::clamp(tangentImpulse, -maxFriction, maxFriction);
				// std::cout << "after clamp tangentImpulse: " << tangentImpulse << "\n";
			
			}
			else
			{
				tangent = glm::vec3(0.0f);
				tangentImpulse = 0.0f;
			}


			// std::cout << "GO!!!!!!!!!!!!!\n";
			finalLinearVelocityA -= velocityConstraint.invMassA * normalImpulse * manifoldPoint.normal;
			finalLinearVelocityB += velocityConstraint.invMassB * normalImpulse * manifoldPoint.normal;
			finalLinearVelocityA += velocityConstraint.invMassA * std::abs(tangentImpulse) * tangent;
			finalLinearVelocityB -= velocityConstraint.invMassB * std::abs(tangentImpulse) * tangent;

			// glm::vec3 plusLinear = -normalImpulse * velocityConstraint.invMassA * manifoldPoint.normal;
			// glm::vec3 plusLinearF = velocityConstraint.invMassA * std::abs(tangentImpulse) * tangent;
			// std::cout << "A normal Linear +: " << plusLinear.x << " " << plusLinear.y << " " << plusLinear.z << "\n";
			// std::cout << "A normal LinearF +: " << plusLinearF.x << " " << plusLinearF.y << " " << plusLinearF.z
			// 		  << "\n";

			finalAngularVelocityA -= velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal);
			finalAngularVelocityB += velocityConstraint.invIB * glm::cross(rB, normalImpulse * manifoldPoint.normal);
			finalAngularVelocityA += velocityConstraint.invIA * glm::cross(rA, std::abs(tangentImpulse) * tangent);
			finalAngularVelocityB -= velocityConstraint.invIB * glm::cross(rB, std::abs(tangentImpulse) * tangent);

			// glm::vec3 plusAngular = -velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal);
			// glm::vec3 plusAngularT = velocityConstraint.invIA * glm::cross(rA, std::abs(tangentImpulse) * tangent) * 1.0f;
			// std::cout << "Angular +: " << plusAngular.x << " " << plusAngular.y << " " << plusAngular.z << "\n";
			// std::cout << "AngularT +: " << plusAngularT.x << " " << plusAngularT.y << " " << plusAngularT.z << "\n";

			// std::cout << "normal adjust angularVelocityA: " << m_velocities[indexA].angularVelocity.x << " "
			// 		  << m_velocities[indexA].angularVelocity.y << " " << m_velocities[indexA].angularVelocity.z
			// 		  << "\n";
			// std::cout << "normal adjust angularVelocityB: " << m_velocities[indexB].angularVelocity.x << " "
			// 		  << m_velocities[indexB].angularVelocity.y << " " << m_velocities[indexB].angularVelocity.z
			// 		  << "\n";

			// std::cout << "tangent adjust angularVelocityA: " << m_velocities[indexA].angularVelocity.x << " "
			// 		  << m_velocities[indexA].angularVelocity.y << " " << m_velocities[indexA].angularVelocity.z
			// 		  << "\n";
			// std::cout << "tangent adjust angularVelocityB: " << m_velocities[indexB].angularVelocity.x << " "
			// 		  << m_velocities[indexB].angularVelocity.y << " " << m_velocities[indexB].angularVelocity.z
			// 		  << "\n";

			// std::cout << "normalA power: "
			// 		  << (velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal)).x << " "
			// 		  << (velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal)).y << " "
			// 		  << (velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal)).z << "\n";
			// std::cout << "normalB power: "
			// 		  << (velocityConstraint.invIB * glm::cross(rB, normalImpulse * manifoldPoint.normal)).x << " "
			// 		  << (velocityConstraint.invIB * glm::cross(rB, normalImpulse * manifoldPoint.normal)).y << " "
			// 		  << (velocityConstraint.invIB * glm::cross(rB, normalImpulse * manifoldPoint.normal)).z << "\n";

			// std::cout << "tangentA power: "
			// 		  << (velocityConstraint.invIA * glm::cross(rA, std::abs(tangentImpulse) * tangent)).x << " "
			// 		  << (velocityConstraint.invIA * glm::cross(rA, std::abs(tangentImpulse) * tangent)).y << " "
			// 		  << (velocityConstraint.invIA * glm::cross(rA, std::abs(tangentImpulse) * tangent)).z << "\n";
			// std::cout << "tangentB power: "
			// 		  << (velocityConstraint.invIB * glm::cross(rB, std::abs(tangentImpulse) * tangent)).x << " "
			// 		  << (velocityConstraint.invIB * glm::cross(rB, std::abs(tangentImpulse) * tangent)).y << " "
			// 		  << (velocityConstraint.invIB * glm::cross(rB, std::abs(tangentImpulse) * tangent)).z << "\n";
		}
		m_velocities[indexA].linearVelocity += finalLinearVelocityA;
		m_velocities[indexB].linearVelocity += finalLinearVelocityB;
		m_velocities[indexA].angularVelocity += finalAngularVelocityA;
		m_velocities[indexB].angularVelocity += finalAngularVelocityB;
		// std::cout << "after velocityA: " << m_velocities[indexA].linearVelocity.x << " "
		// 		  << m_velocities[indexA].linearVelocity.y << " " << m_velocities[indexA].linearVelocity.z << "\n";
		// std::cout << "after velocityB: " << m_velocities[indexB].linearVelocity.x << " "
		// 		  << m_velocities[indexB].linearVelocity.y << " " << m_velocities[indexB].linearVelocity.z << "\n";
		// std::cout << "after angularVelocityA: " << m_velocities[indexA].angularVelocity.x << " "
		// 		  << m_velocities[indexA].angularVelocity.y << " " << m_velocities[indexA].angularVelocity.z << "\n";
		// std::cout << "after angularVelocityB: " << m_velocities[indexB].angularVelocity.x << " "
		// 		  << m_velocities[indexB].angularVelocity.y << " " << m_velocities[indexB].angularVelocity.z << "\n";
	}
}

bool ContactSolver::solvePositionConstraints(const int32_t positionIteration)
{
	const float kSlop = 0.01f; // 허용 관통 오차
	const float alpha = 0.1f / positionIteration;
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
			// std::cout << "seperation: " << manifoldPoint.seperation << "\n";

			glm::vec3 rA = manifoldPoint.pointA - positionConstraint.worldCenterA; // bodyA의 충돌 지점까지의 벡터
			glm::vec3 rB = manifoldPoint.pointB - positionConstraint.worldCenterB; // bodyB의 충돌 지점까지의 벡터
			// 상대 속도 계산
			glm::vec3 relativeVelocity =
				(m_velocities[indexB].linearVelocity + glm::cross(m_velocities[indexB].angularVelocity, rB)) -
				(m_velocities[indexA].linearVelocity + glm::cross(m_velocities[indexA].angularVelocity, rA));
			// 관통 해소된상태면 무시
			if (glm::dot(manifoldPoint.pointA - manifoldPoint.pointB, manifoldPoint.normal) < kSlop)
			{
				// std::cout << "pointA : " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " "
				// 		  << manifoldPoint.pointA.z << "\n";
				// std::cout << "pointB : " << manifoldPoint.pointB.x << " " << manifoldPoint.pointB.y << " "
				// 		  << manifoldPoint.pointB.z << "\n";
				// std::cout << "normal : " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " "
				// 		  << manifoldPoint.normal.z << "\n";

				// std::cout << "Siu!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
				continue;
			}
			// 관통 깊이에 따른 보정량 계산

			float correction = manifoldPoint.seperation * alpha / pointCount;
			glm::vec3 correctionVector = correction * glm::normalize(manifoldPoint.pointA - manifoldPoint.pointB);

			// std::cout << "correction: " << correction << "\n";
			// std::cout << "normal : " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " "
			// 		  << manifoldPoint.normal.z << "\n";
			// std::cout << "correctionVector : " << correctionVector.x << " " << correctionVector.y << " "
			// 		  << correctionVector.z << "\n";

			// 위치 보정
			positionA -= correctionVector * positionConstraint.invMassA /
						 (positionConstraint.invMassA + positionConstraint.invMassB);
			positionB += correctionVector * positionConstraint.invMassB /
						 (positionConstraint.invMassA + positionConstraint.invMassB);
			// std::cout << "pos dA: " << positionA.x << " " << positionA.y << " " << positionA.z << "\n";
			// std::cout << "pos dB: " << positionB.x << " " << positionB.y << " " << positionB.z << "\n";

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
