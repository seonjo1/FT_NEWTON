#ifndef CONTACTSOLVER_H
#define CONTACTSOLVER_H

#include "Island.h"

namespace ale
{

struct ContactPositionConstraint
{
	ManifoldPoint *points;
	int32_t pointCount;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	EManifoldType type;
	bool isStopContact;
};

struct ContactVelocityConstraint
{
	ManifoldPoint *points;
	glm::vec3 worldCenterA;
	glm::vec3 worldCenterB;
	glm::vec3 normal;
	glm::mat3 invIA, invIB;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	float friction;
	float restitution;
	int32_t pointCount;
	bool isStopContact;
};

class ContactSolver
{
  public:
	ContactSolver(float duration, std::vector<Contact *> &contacts, std::vector<Position> &positions,
				  std::vector<Velocity> &velocities);
	void initializeVelocityConstraints();
	void solveVelocityConstraints(const int32_t velocityIteration);
	bool solvePositionConstraints(const int32_t positionIteration);

	float m_duration;
	float m_stopVelocity;
	std::vector<Position> &m_positions;
	std::vector<Velocity> &m_velocities;
	std::vector<ContactPositionConstraint> m_positionConstraints;
	std::vector<ContactVelocityConstraint> m_velocityConstraints;
	std::vector<Contact *> &m_contacts;
};

} // namespace ale

#endif