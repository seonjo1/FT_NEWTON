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
		shape->ComputeAABB(&proxies[i]->aabb);
		proxies[i]->proxyId = broadPhase->CreateProxy(proxies[i]->aabb, proxies[i]);
		proxies[i]->fixture = this;
		proxies[i]->childIndex = i;
	}
}

void Fixture::DestroyProxies(BroadPhase *broadPhase)
{
}

Rigidbody *Fixture::getBody() const
{
	return body;
}

} // namespace ale