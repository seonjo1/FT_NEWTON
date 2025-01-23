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

		// for (int j = 0; j < m_velocityConstraints[i].pointCount; j++)
		// {
		// 	std::cout << "manifoldPoints[" << j << "]: " << &(m_velocityConstraints[i].points[j]) << "\n";
		// 	std::cout << "pointA: " << m_velocityConstraints[i].points[j].pointA.x << " " <<
		// m_velocityConstraints[i].points[j].pointA.y << " " << m_velocityConstraints[i].points[j].pointA.z << "\n";
		// 	std::cout << "pointB: " << m_velocityConstraints[i].points[j].pointB.x << " " <<
		// m_velocityConstraints[i].points[j].pointB.y << " " << m_velocityConstraints[i].points[j].pointB.z << "\n";
		// 	std::cout << "normal: " << m_velocityConstraints[i].points[j].normal.x << " " <<
		// m_velocityConstraints[i].points[j].normal.y << " " << m_velocityConstraints[i].points[j].normal.z << "\n";
		// 	std::cout << "seperation: " << m_velocityConstraints[i].points[j].seperation << "\n";
		// }

		// 위치 제약 설정
		m_positionConstraints[i].worldCenterA = bodyA->getTransform().toMatrix() * glm::vec4(shapeA->m_center, 1.0f);
		m_positionConstraints[i].worldCenterB = bodyB->getTransform().toMatrix() * glm::vec4(shapeB->m_center, 1.0f);
		m_positionConstraints[i].indexA = bodyA->getIslandIndex();
		m_positionConstraints[i].indexB = bodyB->getIslandIndex();
		m_positionConstraints[i].invMassA = bodyA->getInverseMass();
		m_positionConstraints[i].invMassB = bodyB->getInverseMass();
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

void ContactSolver::solveVelocityConstraints(const int32_t velocityIteration)
{
	for (int32_t i = 0; i < m_contactCount; i++)
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

		glm::vec3 &initLinearVelocityA = m_velocities[indexA].linearVelocity;
		glm::vec3 &initLinearVelocityB = m_velocities[indexB].linearVelocity;
		glm::vec3 &initAngularVelocityA = m_velocities[indexA].angularVelocity;
		glm::vec3 &initAngularVelocityB = m_velocities[indexB].angularVelocity;
		glm::vec3 &linearVelocityBufferA = m_velocities[indexA].linearVelocityBuffer;
		glm::vec3 &linearVelocityBufferB = m_velocities[indexB].linearVelocityBuffer;
		glm::vec3 &angularVelocityBufferA = m_velocities[indexA].angularVelocityBuffer;
		glm::vec3 &angularVelocityBufferB = m_velocities[indexB].angularVelocityBuffer;

		linearVelocityBufferA = glm::vec3(0.0f);
		linearVelocityBufferB = glm::vec3(0.0f);
		angularVelocityBufferA = glm::vec3(0.0f);
		angularVelocityBufferB = glm::vec3(0.0f);

		bool isStop = true;

		// std::cout << "bodyA: " << indexA << "\n";
		// std::cout << "bodyB: " << indexB << "\n";
		// std::cout << "bodyA linearVelocity: " << m_velocities[indexA].linearVelocity.x << " " <<
		// m_velocities[indexA].linearVelocity.y << " " << m_velocities[indexA].linearVelocity.z << "\n"; std::cout <<
		// "bodyA angularVelocity: " << m_velocities[indexA].angularVelocity.x << " " <<
		// m_velocities[indexA].angularVelocity.y << " " << m_velocities[indexA].angularVelocity.z << "\n"; std::cout <<
		// "bodyB linearVelocity: " << m_velocities[indexB].linearVelocity.x << " " <<
		// m_velocities[indexB].linearVelocity.y << " " << m_velocities[indexB].linearVelocity.z << "\n"; std::cout <<
		// "bodyB angularVelocity: " << m_velocities[indexB].angularVelocity.x << " " <<
		// m_velocities[indexB].angularVelocity.y << " " << m_velocities[indexB].angularVelocity.z << "\n";
		for (int32_t j = 0; j < pointCount; j++)
		{
			ManifoldPoint &manifoldPoint = velocityConstraint.points[j];

			// std::cout << "manifoldPoint: " << &(velocityConstraint.points[j]) << "\n";
			// std::cout << "pointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " "
			// 		  << manifoldPoint.pointA.z << "\n";
			// std::cout << "pointB: " << (velocityConstraint.points[j].pointB.x) << " " <<
			// (velocityConstraint.points[j].pointB.y) << " "
			// 		  << (velocityConstraint.points[j].pointB.z) << "\n";
			// std::cout << "normal: " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " "
			// 		  << manifoldPoint.normal.z << "\n";
			// std::cout << "seperation: " << manifoldPoint.seperation << "\n";
			glm::vec3 rA = manifoldPoint.pointA - velocityConstraint.worldCenterA; // bodyA의 충돌 지점까지의 벡터
			glm::vec3 rB = manifoldPoint.pointB - velocityConstraint.worldCenterB; // bodyB의 충돌 지점까지의 벡터
			// std::cout << "RARARA pointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " " <<
			// manifoldPoint.pointA.z << "\n"; std::cout << "worldCenterA: " << velocityConstraint.worldCenterA.x << " "
			// << velocityConstraint.worldCenterA.y << " " << velocityConstraint.worldCenterA.z << "\n"; std::cout <<
			// "worldCenterB: " << velocityConstraint.worldCenterB.x << " " << velocityConstraint.worldCenterB.y << " "
			// << velocityConstraint.worldCenterB.z << "\n"; std::cout << "rB: " << rB.x << " " << rB.y << " " << rB.z
			// << "\n";

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
			float normalImpulseForFriction = normalImpulse;
			// std::cout << "m_stopVelocity: " << m_stopVelocity << "\n";
			// std::cout << "normalSpeed: " << normalSpeed << "\n";
			if (normalSpeed > -NORMAL_STOP_VELOCITY)
			{
				normalImpulse = 0.0f;
				if (normalSpeed > NORMAL_STOP_VELOCITY)
				{
					continue;
				}
			}
			else
			{
				// std::cout << "normal not stop!!\n";
			}
			// std::cout << "normalImpulse: " << normalImpulse << "\n";

			glm::vec3 tangent(0.0f);
			glm::vec3 tangentVelocity = relativeVelocity - (normalSpeed * manifoldPoint.normal);
			float tangentSpeed = glm::length(tangentVelocity);
			float tangentImpulse;
			// std::cout << "tangentVelocity: " << tangentVelocity.x << " " << tangentVelocity.y << " "
			// 		  << tangentVelocity.z << "\n";
			// std::cout << "tangentSpeed: " << tangentSpeed << "\n";
			if (tangentSpeed > TANGENT_STOP_VELOCITY)
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
				// std::cout << "tangent not stop!!\n";
			}
			else
			{
				tangentImpulse = 0.0f;
			}

			// std::cout << "GO!!!!!!!!!!!!!\n";

			// glm::vec3 plusLinear = -normalImpulse * velocityConstraint.invMassA * manifoldPoint.normal;
			// glm::vec3 plusLinearF = velocityConstraint.invMassA * std::abs(tangentImpulse) * tangent;
			// std::cout << "A normal Linear +: " << plusLinear.x << " " << plusLinear.y << " " << plusLinear.z << "\n";
			// std::cout << "A normal LinearF +: " << plusLinearF.x << " " << plusLinearF.y << " " << plusLinearF.z
			// 		  << "\n";

			if (normalImpulse != 0)
			{
				linearVelocityBufferA -= velocityConstraint.invMassA * normalImpulse * manifoldPoint.normal;
				linearVelocityBufferB += velocityConstraint.invMassB * normalImpulse * manifoldPoint.normal;
				angularVelocityBufferA -=
					velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal);
				angularVelocityBufferB +=
					velocityConstraint.invIB * glm::cross(rB, normalImpulse * manifoldPoint.normal);
			}

			if (tangentImpulse != 0)
			{
				linearVelocityBufferA += velocityConstraint.invMassA * std::abs(tangentImpulse) * tangent;
				linearVelocityBufferB -= velocityConstraint.invMassB * std::abs(tangentImpulse) * tangent;
				angularVelocityBufferA += velocityConstraint.invIA * glm::cross(rA, std::abs(tangentImpulse) * tangent);
				angularVelocityBufferB -= velocityConstraint.invIB * glm::cross(rB, std::abs(tangentImpulse) * tangent);
			}

			if (std::abs(normalSpeed) > NORMAL_SLEEP_VELOCITY || tangentSpeed > TANGENT_SLEEP_VELOCITY)
			{
				// std::cout << "non stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
				// std::cout << "normalSpeed: " << std::abs(normalSpeed) << "\n";
				// std::cout << "tangentSpeed: " << tangentSpeed << "\n";
				isStop = false;
			}

			// glm::vec3 plusAngular = -velocityConstraint.invIA * glm::cross(rA, normalImpulse * manifoldPoint.normal);
			// glm::vec3 plusAngularT = velocityConstraint.invIA * glm::cross(rA, std::abs(tangentImpulse) * tangent)
			// * 1.0f; std::cout << "Angular +: " << plusAngular.x << " " << plusAngular.y << " " << plusAngular.z <<
			// "\n"; std::cout << "AngularT +: " << plusAngularT.x << " " << plusAngularT.y << " " << plusAngularT.z <<
			// "\n";

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

		if (isStop == false)
		{
			m_positions[indexA].isStop = false;
			m_positions[indexB].isStop = false;
		}

		m_velocities[indexA].linearVelocity += linearVelocityBufferA;
		m_velocities[indexA].angularVelocity += angularVelocityBufferA;
		m_velocities[indexB].linearVelocity += linearVelocityBufferB;
		m_velocities[indexB].angularVelocity += angularVelocityBufferB;

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

void ContactSolver::solvePositionConstraints(const int32_t positionIteration)
{
	const float kSlop = 0.001f; // 허용 관통 오차
	const float alpha = 1.0f / positionIteration;

	for (int i = 0; i < m_contactCount; ++i)
	{
		Contact *contact = m_contacts[i];
		ContactPositionConstraint &positionConstraint = m_positionConstraints[i];
		int32_t pointCount = positionConstraint.pointCount;
		int32_t indexA = positionConstraint.indexA;
		int32_t indexB = positionConstraint.indexB;
		glm::vec3 &positionBufferA = m_positions[indexA].positionBuffer;
		glm::vec3 &positionBufferB = m_positions[indexB].positionBuffer;

		float seperationSum = 0;
		for (int32_t j = 0; j < pointCount; j++)
		{
			seperationSum += positionConstraint.points[j].seperation;
		}

		if (seperationSum < 1e-8)
		{
			seperationSum = 1.0f;
		}

		for (int32_t j = 0; j < pointCount; j++)
		{

			ManifoldPoint &manifoldPoint = positionConstraint.points[j];
			// std::cout << "seperation: " << manifoldPoint.seperation << "\n";

			glm::vec3 &pointA = manifoldPoint.pointA;
			glm::vec3 &pointB = manifoldPoint.pointB;
			float seperation = glm::dot((pointA + positionBufferA) - (pointB + positionBufferB), manifoldPoint.normal);

			// 관통 해소된상태면 무시
			if (seperation < kSlop)
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

			float correction = manifoldPoint.seperation * alpha * (manifoldPoint.seperation / seperationSum);
			glm::vec3 correctionVector = correction * manifoldPoint.normal;

			// std::cout << "correction: " << correction << "\n";
			// std::cout << "normal : " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " "
			// 		  << manifoldPoint.normal.z << "\n";
			// std::cout << "correctionVector : " << correctionVector.x << " " << correctionVector.y << " "
			// 		  << correctionVector.z << "\n";

			// 위치 보정
			positionBufferA -= correctionVector * positionConstraint.invMassA /
							   (positionConstraint.invMassA + positionConstraint.invMassB);
			positionBufferB += correctionVector * positionConstraint.invMassB /
							   (positionConstraint.invMassA + positionConstraint.invMassB);
			// std::cout << "pos dA: " << positionA.x << " " << positionA.y << " " << positionA.z << "\n";
			// std::cout << "pos dB: " << positionB.x << " " << positionB.y << " " << positionB.z << "\n";
		}
	}
}

} // namespace ale
