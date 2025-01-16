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
}

void Fixture::create(Rigidbody *body, const FixtureDef *fd)
{
	// std::cout << "Fixture::Create\n";
	m_shape = fd->shape;
	m_friction = fd->friction;
	m_restitution = fd->restitution;
	this->m_body = body;

	int32_t childCount = m_shape->getChildCount();
	m_proxies.resize(childCount);
	// std::cout << "Fixture::Create - child count: " << childCount << '\n';
	for (int32_t i = 0; i < childCount; ++i)
	{
		if (!m_proxies[i])
			m_proxies[i] = new FixtureProxy();
		m_proxies[i]->fixture = nullptr;
		m_proxies[i]->proxyId = -1;
	}
}

void Fixture::destroy()
{
	int32_t childCount = m_shape->getChildCount();
	for (int32_t i = 0; i < childCount; ++i)
	{
		m_proxies[i]->fixture = nullptr;
		// delete userData
		delete (m_proxies[i]);
	}
	delete m_shape;
}

void Fixture::createProxies(BroadPhase *broadPhase)
{
	// std::cout << "Fixture::Create Proxies\n";
	for (int32_t i = 0; i < m_proxies.size(); ++i)
	{
		m_shape->computeAABB(&m_proxies[i]->aabb, m_body->getTransform());
		m_proxies[i]->proxyId = broadPhase->createProxy(m_proxies[i]->aabb, m_proxies[i]);
		m_proxies[i]->fixture = this;
		m_proxies[i]->childIndex = i;
	}
}

void Fixture::destroyProxies(BroadPhase *broadPhase)
{
}

void Fixture::synchronize(BroadPhase *broadPhase, const Transform &xf1, const Transform &xf2)
{
	if (m_proxies.size() == 0)
	{
		return;
	}

	for (FixtureProxy *proxy : m_proxies)
	{
		AABB aabb1, aabb2;
		m_shape->computeAABB(&aabb1, xf1);
		m_shape->computeAABB(&aabb2, xf2);

		proxy->aabb.combine(aabb1, aabb2);

		glm::vec3 displacement = xf2.position - xf1.position;
		broadPhase->moveProxy(proxy->proxyId, proxy->aabb, displacement);
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
	return m_proxies[0];
}

} // namespace ale