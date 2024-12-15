#include "physics/Shape.h"

namespace ale
{

int32_t operator|(Type type1, Type type2)
{
	return static_cast<int32_t>(type1) | static_cast<int32_t>(type2);
}

} // namespace ale