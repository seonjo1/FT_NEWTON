#ifndef PLANESHAPE_H
#define PLANESHAPE_H

#include "Shape.h"

namespace ale
{
class PlaneShape : public Shape
{
  public:
	glm::vec3 normal;
}
}; // namespace ale

#endif