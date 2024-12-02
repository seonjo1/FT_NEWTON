#ifndef FIXTURE_H
#define FIXTURE_H

#include "Rigidbody.h"
#include "common.h"

namespace ale
{

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
	const std::unique_ptr<Shape> shape;
	void *userData;
	float friction;
	float restitution;
	float density;
};

struct FixtureProxy
{
	AABB aabb;
	Fixture *fixture;
	int32_t childIndex;
	int32_t proxyId;
};

class Fixture
{
  public:
	Fixture();
	void Create(const FixtureDef *fd);
	void Destroy();

	void CreateProxies(BroadPhase *broadPhase);
	void DestroyProxies(BroadPhase *broadPhase);

  protected:
	Rigidbody *body;
	Shape *shape;
	float density;
	float friction;
	float restitution;
	std::vector<std::unique_ptr<FixtureProxy>> proxies;
	// void *userData;

  private:
};
} // namespace ale
#endif