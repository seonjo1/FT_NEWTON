#include "Fixture.h"
#include "BroadPhase.h"

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

void Fixture::Create(const FixtureDef *fd)
{
	shape = def->shape;
	density = def->density;
	friction = def->friction;
	restitution = def->restitution;

	int32_t childCount = shape->GetChildCount();
	proxies.resize(childCount);
	for (int32_t i = 0; i < childCount; ++i)
	{
		proxies[i]->fixture = nullptr;
		proxies[i]->proxyId = -1;
	}
}

void Fixture::Destroy()
{
}

void Fixture::CreateProxies(BroadPhase *broadPhase)
{
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

} // namespace ale