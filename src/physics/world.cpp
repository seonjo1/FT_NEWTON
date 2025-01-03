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
	// std::cout << "start runPhysics\n";
	float duration = 0.0007f;
	for (Rigidbody *body : rigidbodies)
	{
		body->calculateForceAccum();

		body->integrate(duration);

		body->synchronizeFixtures();
	}

	// std::cout << "broad phase\n";
	// update Possible Contact Pairs - BroadPhase
	contactManager.findNewContacts();

	// std::cout << "narrow phase\n";
	// Process Contacts
	contactManager.collide();
	// std::cout << "solve\n";
	solve(duration);
	
	for (Rigidbody *body : rigidbodies)
	{
		app.setTransformById(body->getTransformId(), body->getTransform());
	}
}

void World::solve(float duration)
{
	// std::cout << "solve start\n";
	// island 초기화
	Island island;

	// 모든 body들의 플래그에 islandFlag 제거
	for (Rigidbody *body : rigidbodies)
	{
		body->unsetFlag(EBodyFlag::ISLAND);
	}

	// 모든 contact들의 플래그에 islandFlag 제거
	for (Contact *contact = contactManager.m_contactList; contact; contact = contact->getNext())
	{
		contact->unsetFlag(EContactFlag::ISLAND);
	}

	// Body를 순회하며 island를 생성후 solve 처리
	std::stack<Rigidbody *> stack;

	// body 순회
	for (Rigidbody *body : rigidbodies)
	{
		// std::cout << "body id: " << body->getBodyId() << "\n";
		// 이미 island에 포함된 경우 continue
		if (body->hasFlag(EBodyFlag::ISLAND))
		{
			continue;
		}

		// staticBody인 경우 continue
		if (body->getType() == BodyType::e_static)
		{
			continue;
		}

		// 현재 Body가 island 생성 가능하다 판단이 끝났으니
		// island clear를 통해 새로운 island 생성
		island.clear();
		stack.push(body);
		body->setFlag(EBodyFlag::ISLAND); // body island 처리

		// DFS로 island 생성
		while (!stack.empty())
		{
			// 스택 가장 마지막에 있는 body island에 추가
			Rigidbody *targetBody = stack.top();
			stack.pop();
			island.add(targetBody);

			// body가 staticBody면 뒤에 과정 pass
			if (targetBody->getType() == BodyType::e_static)
			{
				continue;
			}

			// body contactList의 contact들을 island에 추가
			for (ContactLink *link = targetBody->getContactLinks(); link; link = link->next)
			{
				Contact *contact = link->contact;

				// 이미 island에 포함된 경우 continue
				if (contact->hasFlag(EContactFlag::ISLAND))
				{
					continue;
				}

				// contact가 touching 상태가 아니면 continue
				if (contact->hasFlag(EContactFlag::TOUCHING) == false)
				{
					continue;
				}
				// std::cout << "add Contact\n";
				// std::cout << "bodyA : " << contact->getNodeB()->other->getBodyId() << "\n";
				// std::cout << "bodyB : " << contact->getNodeA()->other->getBodyId() << "\n";
				// 위 조건을 다 충족하는 경우 island에 추가 후 island 플래그 on
				island.add(contact);
				contact->setFlag(EContactFlag::ISLAND);

				Rigidbody *other = link->other;

				// 충돌 상대 body가 이미 island에 속한 상태면 continue
				if (other->hasFlag(EBodyFlag::ISLAND))
				{
					continue;
				}

				// 충돌 상대 body가 island에 속한게 아니었으면 stack에 추가 후 island 플래그 on
				stack.push(other);
				other->setFlag(EBodyFlag::ISLAND);
			}
		}

		// 생성한 island 충돌 처리
		island.solve(duration);

		// island의 staticBody들의 island 플래그 off
		for (Rigidbody *body : island.m_bodies)
		{
			if (body->getType() == BodyType::e_static)
			{
				body->unsetFlag(EBodyFlag::ISLAND);
			}
		}
	}
	// std::cout << "finish solve\n\n\n";
}

void World::createBody(std::unique_ptr<Model> &model, int32_t xfId)
{
	// std::cout << "World::Create Body\n";
	Shape *shape = model->getShape();
	Type type = shape->getType();

	switch (type)
	{
	case Type::BOX:
		createBox(model, xfId);
		break;
	case Type::SPHERE:
		createSphere(model, xfId);
		break;
	case Type::GROUND:
		createGround(model, xfId);
		break;
	default:
		break;
	}
}

void World::createBox(std::unique_ptr<Model> &model, int32_t xfId)
{
	// std::cout << "World::Create Box\n";
	Shape *s = model->getShape();
	BoxShape *shape = dynamic_cast<BoxShape *>(s);
	BodyDef bd;

	bd.type = BodyType::e_dynamic;

	bd.position = app.getTransformById(xfId).position;
	bd.orientation = app.getTransformById(xfId).orientation;
	bd.xfId = xfId;
	bd.linearDamping = 0.0001f;
	bd.angularDamping = 0.0001f;

	Rigidbody *body = new Rigidbody(&bd, this);

	// calculate inersiaTensor
	glm::vec3 upper = *std::prev(shape->m_vertices.end());
	glm::vec3 lower = *shape->m_vertices.begin();
	glm::vec3 diff = upper - lower;
	float mass = 30.0f;
	float h = abs(diff.y);
	float w = abs(diff.x);
	float d = abs(diff.z);
	float Ixx = (1.0f / 12.0f) * (h * h + d * d) * mass;
	float Iyy = (1.0f / 12.0f) * (w * w + d * d) * mass;
	float Izz = (1.0f / 12.0f) * (w * w + h * h) * mass;
	glm::mat3 m(glm::vec3(Ixx, 0.0f, 0.0f), glm::vec3(0.0f, Iyy, 0.0f), glm::vec3(0.0f, 0.0f, Izz));

	body->setMassData(mass, m);

	// BoxShape *box = shape->clone();
	BoxShape *box = shape;
	FixtureDef fd;
	fd.shape = shape;
	fd.friction = 0.6f;
	fd.restitution = 0.4f;

	body->createFixture(&fd);
	rigidbodies.push_back(body);
}

void World::createSphere(std::unique_ptr<Model> &model, int32_t xfId)
{
	// std::cout << "World::Create Sphere\n";
	Shape *s = model->getShape();
	SphereShape *shape = dynamic_cast<SphereShape *>(s);

	BodyDef bd;
	// set sphere definition
	bd.type = BodyType::e_dynamic;
	bd.position = app.getTransformById(xfId).position;
	bd.orientation = app.getTransformById(xfId).orientation;
	bd.xfId = xfId;
	bd.linearDamping = 0.0001f;
	bd.angularDamping = 0.0001f;

	Rigidbody *body = new Rigidbody(&bd, this);

	// calculate inersiaTensor - (2 / 5) * m * r * r
	float mass = 10.0f;
	float val = (2.0f / 5.0f) * mass * 1;
	glm::mat3 m(glm::vec3(val, 0.0f, 0.0f), glm::vec3(0.0f, val, 0.0f), glm::vec3(0.0f, 0.0f, val));
	body->setMassData(mass, m);

	// SphereShape *sphere = shape->clone();
	SphereShape *sphere = shape;

	FixtureDef fd;
	fd.shape = shape;
	fd.friction = 0.2f;
	fd.restitution = 0.8f;
	body->createFixture(&fd);
	rigidbodies.push_back(body);
	// std::cout << "World:: Create Sphere end\n";
}

void World::createGround(std::unique_ptr<Model> &model, int32_t xfId)
{
	// std::cout << "World::Create Box\n";
	Shape *s = model->getShape();
	BoxShape *shape = dynamic_cast<BoxShape *>(s);
	BodyDef bd;

	// set box definition
	bd.type = BodyType::e_static;

	bd.position = app.getTransformById(xfId).position;
	bd.orientation = app.getTransformById(xfId).orientation;
	bd.xfId = xfId;
	bd.linearDamping = 0.0f;
	bd.angularDamping = 0.0f;

	Rigidbody *body = new Rigidbody(&bd, this);

	// calculate inersiaTensor
	glm::mat3 m(0.0f);

	float mass = 0;
	body->setMassData(mass, m);

	// BoxShape *box = shape->clone();
	BoxShape *box = shape;
	FixtureDef fd;
	fd.shape = shape;
	fd.friction = 0.7f;
	fd.restitution = 0.3f;

	body->createFixture(&fd);
	rigidbodies.push_back(body);
}

void World::registerBodyForce(int32_t idx, const glm::vec3 &force)
{
	// check idx
	rigidbodies[idx]->registerForce(force);
}

} // namespace ale