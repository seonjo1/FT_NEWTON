#ifndef SHAPE_H
#define SHAPE_H

#include "Vertex.h"
#include "physics/Collision.h"
#include <algorithm>

namespace ale
{
enum class Type
{
	SPHERE = (1 << 0),
	BOX = (1 << 1),
	GROUND = (1 << 2),
};

int32_t operator|(Type type1, Type type2);

struct MassData
{
	glm::vec3 center;
	float mass;
	// inertia
	float I;
};

class Shape
{
  public:
	virtual ~Shape() = default;
	virtual Shape *clone() const = 0;
	virtual int32_t getChildCount() const = 0;
	virtual void computeAABB(AABB *aabb, const Transform &xf) const = 0;
	virtual void computeMass(MassData *massData, float density) const = 0;
	virtual float getLocalRadius() const = 0;
	virtual const glm::vec3 &getLocalHalfSize() const = 0;

	Type getType() const
	{
		return m_type;
	}
	void setType(Type type)
	{
		m_type = type;
	}

	glm::vec3 m_center;
	glm::vec3 localCenter;
	Type m_type;
	bool isCollide {false}; // tmp variable
};

} // namespace ale

#endif