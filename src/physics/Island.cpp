#include "physics/Island.h"

namespace ale
{

const int32_t Island::VELOCITY_ITERATION = 6;
const int32_t Island::POSITION_ITERATION = 2;
const float Island::MAX_TRANSLATION = 2.0f;
const float Island::MAX_ROTATION = 0.5f * glm::pi<float>();
const float Island::MAX_TRANSLATION_SQUARED = Island::MAX_TRANSLATION * Island::MAX_TRANSLATION;
const float Island::MAX_ROTATION_SQUARED = Island::MAX_ROTATION * Island::MAX_ROTATION;

void Island::solve(float duration)
{
	int32_t bodyLength = m_bodies.size();
	m_positions.resize(bodyLength);
	m_velocities.resize(bodyLength);

	// 힘을 적용하여 속도, 위치, 회전 업데이트
	for (int32_t i = 0; i < bodyLength; i++)
	{
		Rigidbody *body = m_bodies[i];

		// 위치, 각도, 속도 기록
		m_positions[i].position = body->getPosition();
		m_positions[i].orientation = body->getOrientation();
		m_velocities[i].linearVelocity = body->getLinearVelocity();
		m_velocities[i].angularVelocity = body->getAngularVelocity();
	}

	ContactSolver contactSolver(duration, m_contacts, m_positions, m_velocities);
	contactSolver.initializeVelocityConstraints();

	// 기존 충돌의 정보로 충격량 계산
	contactSolver.warmStart();

	// 속도 제약 반복 횟수만큼 반복
	for (int32_t i = 0; i < VELOCITY_ITERATION; ++i)
	{
		// 충돌 속도 제약 해결
		contactSolver.solveVelocityConstraints();
	}

	// warm starting을 위해 충격량 기록
	contactSolver.storeImpulses();

	// 위치, 속도 업데이트
	for (int32_t i = 0; i < bodyLength; ++i)
	{
		glm::vec3 position = m_positions[i].position;
		glm::quat orientation = m_positions[i].orientation;
		glm::vec3 linearVelocity = m_velocities[i].linearVelocity;
		glm::vec3 angularVelocity = m_velocities[i].angularVelocity;

		// 이동량이 너무 커지면 작게 조절
		glm::vec3 translation = duration * linearVelocity;
		if (glm::dot(translation, translation) > MAX_TRANSLATION_SQUARED)
		{
			float ratio = MAX_TRANSLATION / glm::length(translation);
			linearVelocity *= ratio;
		}

		// 회전량이 너무 커지면 작게 조절
		float rotation = duration * glm::length(angularVelocity);
		if (rotation * rotation > MAX_ROTATION_SQUARED)
		{
			float ratio = MAX_ROTATION / std::abs(rotation);
			angularVelocity *= ratio;
		}

		// 위치, 회전에 속도 적용
		position += duration * linearVelocity;
		glm::quat angularVelocityQuat = glm::quat(0.0f, angularVelocity * duration); // 각속도를 쿼터니언으로 변환
		orientation += 0.5f * angularVelocityQuat * orientation;					 // 쿼터니언 미분 공식
		orientation = glm::normalize(orientation);

		// 업데이트
		m_positions[i].position = position;
		m_positions[i].orientation = orientation;
		m_velocities[i].linearVelocity = linearVelocity;
		m_velocities[i].angularVelocity = angularVelocity;
	}

	bool positionSolved = false;
	// 위치 제약 처리 반복
	for (int32_t i = 0; i < POSITION_ITERATION; ++i)
	{
		bool contactsOkay = contactSolver.solvePositionConstraints();

		// 위치 제약 해제시 반복문 탈출
		if (contactsOkay)
		{
			positionSolved = true;
			break;
		}
	}

	// 위치, 회전, 속도 업데이트
	for (int32_t i = 0; i < bodyLength; ++i)
	{
		Rigidbody *body = m_bodies[i];
		body->updateSweep();
		body->setPosition(m_positions[i].position);
		body->setOrientation(m_positions[i].orientation);
		body->setLinearVelocity(m_velocities[i].linearVelocity);
		body->setAngularVelocity(m_velocities[i].angularVelocity);
		body->synchronizeFixtures();
	}
}

void Island::add(Rigidbody *body)
{
	m_bodies.push_back(body);
}

void Island::add(Contact *contact)
{
	m_contacts.push_back(contact);
}

void Island::clear()
{
	m_bodies.clear();
	m_contacts.clear();
	m_positions.clear();
	m_velocities.clear();
}

} // namespace ale