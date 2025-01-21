#ifndef FIXTURE_H
#define FIXTURE_H

#include "Common.h"
#include "physics/Rigidbody.h"
#include "physics/PhysicsAllocator.h"

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
		friction = 0.0f;
		restitution = 0.0f;
	}
	Shape *shape;
	void *userData;
	float friction;
	float restitution;
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
	void create(Rigidbody *body, const FixtureDef *fd);
	void destroy();

	void createProxies(BroadPhase *broadPhase);
	void destroyProxies(BroadPhase *broadPhase);

	void synchronize(BroadPhase *broadPhase, const Transform &xf1, const Transform &xf2);

	float getFriction();
	float getRestitution();
	Rigidbody *getBody() const;
	EType getType() const;
	Shape *getShape();
	const FixtureProxy *getFixtureProxy() const;

  protected:
	Rigidbody *m_body;
	Shape *m_shape;
	float m_density;
	float m_friction;
	float m_restitution;

	FixtureProxy *m_proxies;
	int32_t m_proxyCount;
	// void *userData;

  private:
};
} // namespace ale
#endif