#ifndef FIXTURE_H
#define FIXTURE_H

#include "common.h"
#include "physics/Rigidbody.h"

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
	Shape *shape;
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
	void Create(Rigidbody *body, const FixtureDef *fd);
	void Destroy();

	void CreateProxies(BroadPhase *broadPhase);
	void DestroyProxies(BroadPhase *broadPhase);

	void synchronize(BroadPhase *broadPhase, const Transform &xf1, const Transform &xf2);

	float getFriction();
	float getRestitution();
	Rigidbody *getBody() const;
	Type getType() const;
	Shape *getShape();
	const FixtureProxy* getFixtureProxy() const;

  protected:
	Rigidbody *body;
	Shape *shape;
	float density;
	float friction;
	float restitution;
	std::vector<FixtureProxy *> proxies;
	// void *userData;

  private:
};
} // namespace ale
#endif