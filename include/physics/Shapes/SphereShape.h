#ifndef SPHERESHAPE_H
#define SPHERESHAPE_H

#include "Shape.h"

namespace ale
{
class SphereShape : public Shape
{
  public:
	glm::vec3 radius;
}
}; // namespace ale

#endif