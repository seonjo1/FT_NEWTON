#ifndef CONTACTSOLVER_H
#define CONTACTSOLVER_H

#include "Island.h"

namespace ale
{

struct VelocityConstraintPoint
{
	glm::vec3 rA;
	glm::vec3 rB;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float velocityBias;
};

struct ContactPositionConstraint
{
	ManifoldPoint* points;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	glm::vec3 localCenterA, localCenterB;
	glm::mat3 invIA, invIB;
	EManifoldType type;
	float radiusA, radiusB;
	int32_t pointCount;
};

struct ContactVelocityConstraint
{
	ManifoldPoint* points;
	glm::vec3 normal;
	glm::mat3 normalMass;
	glm::mat3 K;
	glm::mat3 invIA, invIB;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	float friction;
	float restitution;
	float tangentSpeed;
	int32_t pointCount;
	int32_t contactIndex;
};

class ContactSolver
{
  public:
	ContactSolver(float duration, std::vector<Contact *> &contacts, std::vector<Position> &positions,
				  std::vector<Velocity> &velocities);
	void initializeVelocityConstraints();
	void solveVelocityConstraints();
	void storeImpulses();
	bool solvePositionConstraints();
	void warmStart();

	float m_duration;
	std::vector<Position> &m_positions;
	std::vector<Velocity> &m_velocities;
	std::vector<ContactPositionConstraint> m_positionConstraints;
	std::vector<ContactVelocityConstraint> m_velocityConstraints;
	std::vector<Contact *> &m_contacts;
};

} // namespace ale

#endif