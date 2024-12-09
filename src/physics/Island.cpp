#include "physics/Island.h"

namespace ale
{
void Island::Solve(float duration)
{
	// contactSolverDef 초기화
	ContactSolverDef contactSolverDef;
	contactSolverDef.duration = duration;
	contactSolverDef.contacts = &m_contacts;
	contactSolverDef.positions = &m_positions;
	contactSolverDef.velocities = &m_velocities;

	ContactSolver contactSolver(&contactSolverDef);
	contactSolver.initializeVelocityConstraints();

	if (step.warmStarting)
	{
		// 기존 충돌의 정보로 충격량 계산
		contactSolver.WarmStart();
	}
	// 속도 제약 반복 횟수만큼 반복
	for (int32_t i = 0; i < EIterations::VELOCITY; ++i)
	{
		// 충돌 속도 제약 해결
		contactSolver.solveVelocityConstraints();
	}

	// warm starting을 위해 충격량 기록
	contactSolver.storeImpulses();

	// 위치, 속도 업데이트
	int32_t size = m_bodies.size();
	for (int32_t i = 0; i < size; ++i)
	{
		glm::vec3 p = m_positions[i].position;
		float q = m_positions[i].orientation;
		glm::vec3 v = m_velocities[i].linearVelocity;
		float w = m_velocities[i].angularVelocity;

		// 이동량이 너무 커지면 작게 조절
		glm::vec3 translation = h * v;
		if (glm::dot(translation, translation) > b2_maxTranslationSquared)
		{
			float ratio = b2_maxTranslation / translation.Length();
			v *= ratio;
		}

		// 회전량이 너무 커지면 작게 조절
		float rotation = h * w;
		if (rotation * rotation > b2_maxRotationSquared)
		{
			float ratio = b2_maxRotation / std::abs(rotation);
			w *= ratio;
		}

		// 위치, 회전에 속도 적용
		p += h * v;
		q += h * w;

		// 업데이트
		m_positions[i].position = p;
		m_positions[i].orientation = q;
		m_velocities[i].linearVelocity = v;
		m_velocities[i].angularVelocity = w;
	}

	bool positionSolved = false;
	// 위치 제약 처리 반복
	for (int32_t i = 0; i < EIterations::POSITION; ++i)
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
	for (int32_t i = 0; i < size; ++i)
	{
		Rigidbody *body = m_bodies[i];
		body->m_sweep.p = m_positions[i].p;
		body->m_sweep.q = m_positions[i].q;
		body->m_linearVelocity = m_velocities[i].v;
		body->m_angularVelocity = m_velocities[i].w;
		body->synchronizeTransform();
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