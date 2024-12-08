#include "physics/Fixture.h"
#include "physics/BroadPhase.h"
#include "physics/Rigidbody.h"

namespace ale
{
Fixture::Fixture()
{
	body = nullptr;
	shape = nullptr;
	density = 0.0f;
	friction = 0.0f;
	restitution = 0.0f;
}

void Fixture::Create(Rigidbody *body, const FixtureDef *fd)
{
	std::cout << "Fixture::Create\n";
	shape = fd->shape;
	density = fd->density;
	friction = fd->friction;
	restitution = fd->restitution;
	this->body = body;

	int32_t childCount = shape->GetChildCount();
	proxies.resize(childCount);
	std::cout << "Fixture::Create - child count: " << childCount << '\n';
	for (int32_t i = 0; i < childCount; ++i)
	{
		if (!proxies[i])
			proxies[i] = new FixtureProxy();
		proxies[i]->fixture = nullptr;
		proxies[i]->proxyId = -1;
	}
}

void Fixture::Destroy()
{
}

void Fixture::CreateProxies(BroadPhase *broadPhase)
{
	std::cout << "Fixture::Create Proxies\n";
	for (int32_t i = 0; i < proxies.size(); ++i)
	{
		shape->ComputeAABB(&proxies[i]->aabb, body->getTransform());
		proxies[i]->proxyId = broadPhase->CreateProxy(proxies[i]->aabb, proxies[i]);
		proxies[i]->fixture = this;
		proxies[i]->childIndex = i;
	}
}

void Fixture::DestroyProxies(BroadPhase *broadPhase)
{
}

void Fixture::synchronize(BroadPhase *broadPhase, const Transform &xf1, const Transform &xf2)
{
	if (proxies.size() == 0)
	{
		return;
	}

	for (FixtureProxy *proxy : proxies)
	{
		AABB aabb1, aabb2;
		shape->ComputeAABB(&aabb1, xf1);
		shape->ComputeAABB(&aabb2, xf2);

		proxy->aabb.Combine(aabb1, aabb2);

		glm::vec3 displacement = xf2.position - xf1.position;
		broadPhase->MoveProxy(proxy->proxyId, proxy->aabb, displacement);
	}
}

Rigidbody *Fixture::getBody() const
{
	return body;
}

} // namespace ale