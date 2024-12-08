#ifndef SHAPE_H
#define SHAPE_H

#include "common.h"

namespace ale
{
enum class EShapeType
{
	SHPERE = 0,
	BOX = 1,
	PLANE = 2,
};

class Shape
{
  public:
	EShapeType m_type;
	glm::vec3 pos;
};
} // namespace ale
#endif