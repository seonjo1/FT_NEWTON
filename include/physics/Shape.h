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
	SPHERE = 0,
	BOX = 1
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
	virtual int32_t getChildCount() const = 0;
	virtual void computeAABB(AABB *aabb, const Transform &xf) const = 0;
	virtual void computeMass(MassData *massData, float density) const = 0;
	Type getType() const
	{
		return m_type;
	}

	glm::vec3 m_center;
	Type m_type;
};

} // namespace ale

#endif