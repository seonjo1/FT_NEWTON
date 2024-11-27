#ifndef FORCEGENERATOR_H
#define FORCEGENERATOR_H

namespace ale
{
class Rigidbody;

class ForceGenerator
{
  public:
	virtual void updateForce(Rigidbody *body, float duration) = 0;
};
} // namespace ale
#endif