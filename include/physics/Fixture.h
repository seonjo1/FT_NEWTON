#ifndef FIXTURE_H
#define FIXTURE_H

#include <glm/glm.hpp>

namespace ale
{

class Shape;
class BroadPhase;
class Rigidbody;

struct FixtureDef
{
	FixtureDef()
	{
		shape = nullptr;
		userData = nullptr;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
	}
	const Shape *shape;
	void *userData;
	float friction;
	float restitution;
	float density;
};

struct FixtureProxy
{
	AABB aabb;
	Fixture *fixture;
	int childIndex;
	int proxyId;
};

class Fixture
{
  public:
	void Create(FixtureDef *def);
	void Destroy();

	void CreateProxies(BroadPhase *broadPhase, const Transform &xf);
	void DestroyProxies(BroadPhase *broadPhase);

  protected:
	Rigidbody *body;
	Shape *shape;
	float density;
	float friction;
	float restitution;
	// std::vector<FixtureProxy> proxies;
	void *userData;

  private:
};
} // namespace ale
#endif