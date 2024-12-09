#ifndef CONTACTSOLVER_H
#define CONTACTSOLVER_H

#include "Rigidbody.h"

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
	std::vector<glm::vec3> localPoints;
	glm::vec3 localNormal;
	glm::vec3 localPoint;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	glm::vec3 localCenterA, localCenterB;
	float invIA, invIB;
	EManifoldType type;
	float radiusA, radiusB;
	int32_t pointCount;
};

struct ContactVelocityConstraint
{
	std::vector<VelocityConstraintPoint> points;
	glm::vec3 normal;
	glm::mat3 normalMass;
	glm::mat3 K;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	float tangentSpeed;
	int32_t pointCount;
	int32_t contactIndex;
};

struct ContactSolverDef
{
	float duration;
	std::vector<Contact *> *contacts;
	std::vector<Position> *positions;
	std::vector<Velocity> *velocities;
};

class ContactSolver
{
  public:
	ContactSolver(ContactSolverDef *def);
	void initializeVelocityConstraints();
	void solveVelocityConstraints();
	void storeImpulses();
	bool solvePositionConstraints();

	float m_duration;
	std::vector<Position *> &m_positions;
	std::vector<Velocity *> &m_velocities;
	std::vector<ContactPositionConstraint> &m_positionConstraints;
	std::vector<ContactVelocityConstraint> &m_velocityConstraints;
	std::vector<Contact *> &m_contacts;
};

} // namespace ale

#endif