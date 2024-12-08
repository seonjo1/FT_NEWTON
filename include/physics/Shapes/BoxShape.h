#ifndef BOXSHAPE_H
#define BOXSHAPE_H

#include "Shape.h"

namespace ale
{
class BoxShape : public Shape
{
  public:
	glm::vec3 halfSize;
}
}; // namespace ale

#endif