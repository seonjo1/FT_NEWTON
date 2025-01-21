#include "physics/Fixture.h"
#include "physics/BroadPhase.h"
#include "physics/Rigidbody.h"

namespace ale
{
Fixture::Fixture()
{
	m_body = nullptr;
	m_shape = nullptr;
	m_density = 0.0f;
	m_friction = 0.0f;
	m_restitution = 0.0f;
	m_proxies = nullptr;
	m_proxyCount = 0;
}

void Fixture::create(Rigidbody *body, const FixtureDef *fd)
{
	// std::cout << "Fixture::Create\n";
	m_shape = fd->shape;
	m_friction = fd->friction;
	m_restitution = fd->restitution;
	this->m_body = body;

	m_proxyCount = m_shape->getChildCount();
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(FixtureProxy) * m_proxyCount);
	m_proxies = static_cast<FixtureProxy *>(memory);

	// std::cout << "Fixture::Create - child count: " << childCount << '\n';
	for (int32_t i = 0; i < m_proxyCount; ++i)
	{
		new (&m_proxies[i]) FixtureProxy();
		m_proxies[i].fixture = nullptr;
		m_proxies[i].proxyId = -1;
	}
}

void Fixture::destroy()
{
	for (int32_t i = 0; i < m_proxyCount; ++i)
	{
		m_proxies[i].fixture = nullptr;
		// delete userData
	}
	m_shape->~Shape();

	PhysicsAllocator::m_blockAllocator.freeBlock(m_proxies, sizeof(FixtureProxy) * m_proxyCount);
	PhysicsAllocator::m_blockAllocator.freeBlock(m_shape, sizeof(Shape));
}

void Fixture::createProxies(BroadPhase *broadPhase)
{
	// std::cout << "Fixture::Create Proxies\n";
	for (int32_t i = 0; i < m_proxyCount; ++i)
	{
		m_shape->computeAABB(&m_proxies[i].aabb, m_body->getTransform());
		m_proxies[i].proxyId = broadPhase->createProxy(m_proxies[i].aabb, &(m_proxies[i]));
		m_proxies[i].fixture = this;
		m_proxies[i].childIndex = i;
	}
}

void Fixture::destroyProxies(BroadPhase *broadPhase)
{
}

void Fixture::synchronize(BroadPhase *broadPhase, const Transform &xf1, const Transform &xf2)
{
	if (m_proxyCount == 0)
	{
		return;
	}

	for (int32_t i = 0; i < m_proxyCount; ++i)
	{
		FixtureProxy &proxy = m_proxies[i];

		AABB aabb1, aabb2;
		m_shape->computeAABB(&aabb1, xf1);
		m_shape->computeAABB(&aabb2, xf2);

		proxy.aabb.combine(aabb1, aabb2);

		glm::vec3 displacement = xf2.position - xf1.position;
		broadPhase->moveProxy(proxy.proxyId, proxy.aabb, displacement);
	}
}

Rigidbody *Fixture::getBody() const
{
	return m_body;
}

EType Fixture::getType() const
{
	return m_shape->getType();
}

Shape *Fixture::getShape()
{
	return m_shape;
}

float Fixture::getFriction()
{
	return m_friction;
}

float Fixture::getRestitution()
{
	return m_restitution;
}

const FixtureProxy *Fixture::getFixtureProxy() const
{
	return m_proxies;
}

} // namespace ale