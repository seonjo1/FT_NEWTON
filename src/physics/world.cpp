#include "physics/world.h"
#include "app.h"
#include "model.h"
#include "physics/BoxShape.h"
#include "physics/Rigidbody.h"
#include "physics/SphereShape.h"

namespace ale
{
World::World(uint32_t size, App &app) : app(app)
{
}

void World::startFrame()
{
	for (Rigidbody *body : rigidbodies)
	{
		body->clearAccumulators();
		body->calculateDerivedData();
	}
}

void World::runPhysics()
{
	for (Rigidbody *body : rigidbodies)
	{
		body->calculateForceAccum();
		body->integrate(0.001f);
		body->synchronizeFixtures();
		app.setTransformById(body->getTransformId(), body->getTransform());
	}
	// update Possible Contact Pairs - BroadPhase
	contactManager.findNewContacts();
	// Process Contacts
	contactManager.collide();
	solve(0.001f);
}

void World::Solve(float duration)
{
	// island 초기화
	Island island;

	// 모든 body들의 플래그에 islandFlag 제거
	for (Rigidbody *body : rigidbodies)
	{
		body->m_flags &= ~EBodyFlag::ISLAND;
	}

	// 모든 contact들의 플래그에 islandFlag 제거
	for (Contact *contact : m_contactManager.m_contactList)
	{
		contact->m_flags &= ~EContactFlag::ISLAND;
	}

	// Body를 순회하며 island를 생성후 solve 처리
	std::vector<Rigidbody *> stack(rigidbodies.size());

	// body 순회
	for (Rigidbody *body : rigidbodies)
	{
		// 이미 island에 포함된 경우 continue
		if (body->m_flags & EBodyFlag::ISLAND)
		{
			continue;
		}

		// staticBody인 경우 continue
		if (body->GetType() == BodyType::e_static)
		{
			continue;
		}

		// 현재 Body가 island 생성 가능하다 판단이 끝났으니
		// island clear를 통해 새로운 island 생성
		island.clear();
		int32_t stackCount = 0;
		stack[stackCount] = body;
		stackCount++;
		body->m_flags |= EBodyFlag::ISLAND; // body island 처리

		// DFS로 island 생성
		while (stackCount > 0)
		{
			// 스택 가장 마지막에 있는 body island에 추가
			stackCount--;
			Rigidbody *targetBody = stack[stackCount];
			island.add(targetBody);

			// body가 staticBody면 뒤에 과정 pass
			if (targetBody->GetType() == BodyType::e_static)
			{
				continue;
			}

			// body contactList의 contact들을 island에 추가
			for (ContactLink *link : targetBody->GetContactLinks)
			{
				Contact *contact = link->contact;

				// 이미 island에 포함된 경우 continue
				if (contact->m_flags & EContactFlag::ISLAND)
				{
					continue;
				}

				// contact가 touching 상태가 아니면 continue
				if (contact->isTouching() == false)
				{
					continue;
				}

				// 위 조건을 다 충족하는 경우 island에 추가 후 island 플래그 on
				island.add(contact);
				contact->m_flags |= EContactFlag::ISLAND;

				Rigidbody *other = link->other;

				// 충돌 상대 body가 이미 island에 속한 상태면 continue
				if (other->m_flags & EBodyFlag::ISLAND)
				{
					continue;
				}

				// 충돌 상대 body가 island에 속한게 아니었으면 stack에 추가 후 island 플래그 on
				stack[stackCount++] = other;
				other->m_flags |= EBodyFlag::ISLAND;
			}
		}

		// 생성한 island 충돌 처리
		island.Solve(&profile, step, m_gravity, m_allowSleep);

		// island의 staticBody들의 island 플래그 off
		for (Rigidbody *body : island.m_bodies)
		{
			if (body->GetType() == BodyType::e_static)
			{
				body->m_flags &= ~EBodyFlag::ISLAND;
			}
		}
	}
}

void World::createBody(std::unique_ptr<Model> &model, int32_t xfId)
{
	std::cout << "World::Create Body\n";
	Shape *shape = model->getShape();
	Type type = shape->getType();

	switch (type)
	{
	case Type::e_box:
		createBox(model, xfId);
		break;
	case Type::e_sphere:
		createSphere(model, xfId);
		break;
	default:
		break;
	}
}

void World::createBox(std::unique_ptr<Model> &model, int32_t xfId)
{
	std::cout << "World::Create Box\n";
	Shape *s = model->getShape();
	BoxShape *shape = dynamic_cast<BoxShape *>(s);
	BodyDef bd;

	// set box definition
	bd.type = BodyType::e_dynamic;

	bd.position = app.getTransformById(xfId).position;
	bd.orientation = app.getTransformById(xfId).orientation;
	bd.xfId = xfId;
	bd.linearDamping = 0.01f;
	bd.angularDamping = 0.01f;

	Rigidbody *body = new Rigidbody(&bd, this);

	// calculate inersiaTensor
	glm::vec3 upper = *std::prev(shape->vertices.end());
	glm::vec3 lower = *shape->vertices.begin();
	glm::vec3 diff = upper - lower;
	float h = abs(diff.y);
	float w = abs(diff.x);
	float d = abs(diff.z);
	float Ixx = (1.0f / 12.0f) * (h * h + d * d);
	float Iyy = (1.0f / 12.0f) * (w * w + d * d);
	float Izz = (1.0f / 12.0f) * (w * w + h * h);
	glm::mat3 m(glm::vec3(Ixx, 0.0f, 0.0f), glm::vec3(0.0f, Iyy, 0.0f), glm::vec3(0.0f, 0.0f, Izz));

	body->setMassData(1, m);

	BoxShape *box = shape->clone();
	body->createFixture(box);
	rigidbodies.push_back(body);
}

void World::createSphere(std::unique_ptr<Model> &model, int32_t xfId)
{
	std::cout << "World::Create Sphere\n";
	Shape *s = model->getShape();
	SphereShape *shape = dynamic_cast<SphereShape *>(s);

	BodyDef bd;
	// set sphere definition
	bd.type = BodyType::e_dynamic;
	bd.position = app.getTransformById(xfId).position;
	bd.orientation = app.getTransformById(xfId).orientation;
	bd.xfId = xfId;
	bd.linearDamping = 0.01f;
	bd.angularDamping = 0.01f;

	Rigidbody *body = new Rigidbody(&bd, this);

	// calculate inersiaTensor - (2 / 5) * m * r * r
	float val = (2.0f / 5.0f) * 1 * 1;
	glm::mat3 m(glm::vec3(val, 0.0f, 0.0f), glm::vec3(0.0f, val, 0.0f), glm::vec3(0.0f, 0.0f, val));
	body->setMassData(1, m);

	SphereShape *sphere = shape->clone();
	body->createFixture(sphere);
	rigidbodies.push_back(body);
	std::cout << "World:: Create Sphere end\n";
}

void World::registerBodyForce(int32_t idx, const glm::vec3 &force)
{
	// check idx
	rigidbodies[idx]->registerForce(force);
}

} // namespace ale