#ifndef ISLAND_H
#define ISLAND_H

#include "Contact.h"

namespace ale
{

struct Position
{
	glm::vec3 position;
};

struct Velocity
{
	glm::vec3 linearVelocity;
	glm::vec3 angularVelocity;
};

class Island
{
  public:
	Island() = default;
	~Island() = default;
	void solve(float duration);

	void add(Rigidbody *body);
	void add(Contact *contact);
	void clear();

	static const int32_t VELOCITY_ITERATION;
	static const int32_t POSITION_ITERATION;

	std::vector<Rigidbody *> m_bodies;
	std::vector<Contact *> m_contacts;

	std::vector<Position> m_positions;
	std::vector<Velocity> m_velocities;
};

} // namespace ale

#endif