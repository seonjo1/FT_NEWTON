#ifndef SHAPE_H
#define SHAPE_H

#include "Collision.h"
#include "Vertex.h"
#include "common.h"
#include <algorithm>

namespace ale
{
enum class Type
{
	e_sphere = 0,
	e_box = 1
};

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
	Type getType() const
	{
		return type;
	}

	glm::vec3 center;
	Type type;
};

} // namespace ale

#endif