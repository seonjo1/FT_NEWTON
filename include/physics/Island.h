#ifndef ISLAND_H
#define ISLAND_H

#include "ContactSolver.h"
#include "Rigidbody.h"

namespace ale
{

enum class EIterations
{
	VELOCITY = 6,
	POSITION = 2
}

struct Position
{
	glm::vec3 position;
	glm::vec3 orientation;
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

	void add(Rigidbody *body);
	void add(Contact *contact);
	void clear();

	std::vector<Rigidbody *> m_bodies;
	std::vector<Contact *> m_contacts;

	std::vector<Position> m_positions;
	std::vector<Velocity> m_velocities;
};

} // namespace ale

#endif