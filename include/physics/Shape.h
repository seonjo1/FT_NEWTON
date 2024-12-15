#ifndef SHAPE_H
#define SHAPE_H

#include "Vertex.h"
#include "physics/Collision.h"
#include <algorithm>

namespace ale
{
enum class Type
{
	e_sphere = (1 << 0),
	e_box = (1 << 1)
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
	virtual int32_t GetChildCount() const = 0;
	virtual void ComputeAABB(AABB *aabb, const Transform &xf) const = 0;
	virtual void ComputeMass(MassData *massData, float density) const = 0;
	virtual float getLocalRadius() const = 0;
	virtual const glm::vec3 &getLocalHalfSize() const = 0;

	Type getType() const
	{
		return type;
	}

	glm::vec3 center;
	glm::vec3 localCenter;
	Type type;
};

} // namespace ale

#endif