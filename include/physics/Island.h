#ifndef ISLAND_H
#define ISLAND_H

#include "Contact.h"

namespace ale
{

struct Position
{
	glm::vec3 position;
	glm::vec3 positionBuffer;
	bool isNormalStop {true};
	bool isTangentStop {true};
	bool isNormal {false};
};


struct Velocity
{
	glm::vec3 linearVelocity;
	glm::vec3 angularVelocity;
	glm::vec3 linearVelocityBuffer;
	glm::vec3 angularVelocityBuffer;
};

class Island
{
  public:
	Island(int32_t bodyCount, int32_t contactCount);
	void solve(float duration);
	void destroy();

	void add(Rigidbody *body);
	void add(Contact *contact);
	void clear();

	static const int32_t VELOCITY_ITERATION;
	static const int32_t POSITION_ITERATION;
	static const float STOP_LINEAR_VELOCITY;
	static const float STOP_ANGULAR_VELOCITY;

	Rigidbody **m_bodies;
	Contact **m_contacts;
	Position *m_positions;
	Velocity *m_velocities;

	int32_t m_bodyCount;
	int32_t m_contactCount;
};

} // namespace ale

#endif