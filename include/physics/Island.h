#ifndef ISLAND_H
#define ISLAND_H

#include "Contact.h"

namespace ale
{

struct Position
{
	glm::vec3 position;
	~Position() = default;
};

struct Velocity
{
	glm::vec3 linearVelocity;
	glm::vec3 angularVelocity;
	~Velocity() = default;
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

	Rigidbody **m_bodies;
	Contact **m_contacts;
	Position *m_positions;
	Velocity *m_velocities;

	int32_t m_bodyCount;
	int32_t m_contactCount;
};

} // namespace ale

#endif