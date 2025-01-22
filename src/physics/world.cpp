#include "physics/World.h"
#include "App.h"
#include "Model.h"
#include "physics/BoxShape.h"
#include "physics/Rigidbody.h"
#include "physics/SphereShape.h"

namespace ale
{
World::World(App &app) : m_app(app), m_rigidbodies(nullptr), m_rigidbodyCount(0) {};

World::~World()
{
	Rigidbody *body = m_rigidbodies;

	while (body != nullptr)
	{
		Rigidbody *nextBody = body->next;
		body->~Rigidbody();
		PhysicsAllocator::m_blockAllocator.freeBlock(body, sizeof(Rigidbody));

		body = nextBody;
	}
}

void World::startFrame()
{
	Rigidbody *body = m_rigidbodies;
	while (body != nullptr)
	{
		body->clearAccumulators();
		body->calculateDerivedData();
		body = body->next;
	}
}

void World::runPhysics()
{
	// std::cout << "start runPhysics\n";
	float duration = 0.003f;
	Rigidbody *body = m_rigidbodies;
	while (body != nullptr)
	{
		// std::cout << "body: " << body->getBodyId() << "\n";
		body->calculateForceAccum();

		body->integrate(duration);

		body->synchronizeFixtures();

		body = body->next;
	}

	// std::cout << "broad phase\n";
	// update Possible Contact Pairs - BroadPhase
	m_contactManager.findNewContacts();

	// std::cout << "narrow phase\n";
	// Process Contacts
	m_contactManager.collide();
	// std::cout << "solve\n";
	solve(duration);

	// std::cout << "transform setting\n";

	body = m_rigidbodies;

	while (body != nullptr)
	{
		m_app.setTransformById(body->getTransformId(), body->getTransform());
		body = body->next;
	}
}

void World::solve(float duration)
{
	// std::cout << "solve start\n";
	// island 초기화
	Island island(m_rigidbodyCount, m_contactManager.m_contactCount);

	// 모든 body들의 플래그에 islandFlag 제거
	for (Rigidbody *body = m_rigidbodies; body; body = body->next)
	{
		body->unsetFlag(EBodyFlag::ISLAND);
	}

	// 모든 contact들의 플래그에 islandFlag 제거
	for (Contact *contact = m_contactManager.m_contactList; contact; contact = contact->getNext())
	{
		contact->unsetFlag(EContactFlag::ISLAND);
	}

	// Body를 순회하며 island를 생성후 solve 처리
	Rigidbody **stack = static_cast<Rigidbody **>(PhysicsAllocator::m_stackAllocator.allocateStack(m_rigidbodyCount));
	int32_t stackPtr = 0;

	// body 순회
	for (Rigidbody *body = m_rigidbodies; body; body = body->next)
	{
		// 이미 island에 포함된 경우 continue
		if (body->hasFlag(EBodyFlag::ISLAND))
		{
			continue;
		}

		// staticBody인 경우 continue
		if (body->getType() == EBodyType::STATIC_BODY)
		{
			continue;
		}

		// 현재 Body가 island 생성 가능하다 판단이 끝났으니
		// island clear를 통해 새로운 island 생성
		island.clear();
		stack[stackPtr] = body;
		++stackPtr;
		body->setFlag(EBodyFlag::ISLAND); // body island 처리

		// DFS로 island 생성
		while (stackPtr > 0)
		{
			// 스택 가장 마지막에 있는 body island에 추가
			Rigidbody *targetBody = stack[--stackPtr];
			island.add(targetBody);

			// body가 staticBody면 뒤에 과정 pass
			if (targetBody->getType() == EBodyType::STATIC_BODY)
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
				stack[stackPtr] = other;
				stackPtr++;
				other->setFlag(EBodyFlag::ISLAND);
			}
		}

		// 생성한 island 충돌 처리
		island.solve(duration);

		// island의 staticBody들의 island 플래그 off
		Rigidbody **islandBodies = island.m_bodies;
		for (int32_t i = 0; i < island.m_bodyCount; ++i)
		{

			if (islandBodies[i]->getType() == EBodyType::STATIC_BODY)
			{
				islandBodies[i]->unsetFlag(EBodyFlag::ISLAND);
			}
		}
	}

	island.destroy();

	PhysicsAllocator::m_stackAllocator.freeStack();
	// std::cout << "finish solve\n\n\n";
}

void World::createBody(std::unique_ptr<Model> &model, int32_t xfId)
{
	// std::cout << "World::Create Body\n";
	Shape *shape = model->getShape();
	EType type = shape->getType();

	switch (type)
	{
	case EType::SPHERE:
		createSphere(model, xfId);
		break;
	case EType::BOX:
		createBox(model, xfId);
		break;
	case EType::GROUND:
		createGround(model, xfId);
		break;
	case EType::CYLINDER:
		createCylinder(model, xfId);
		break;
	case EType::CAPSULE:
		createCapsule(model, xfId);
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

	bd.m_type = EBodyType::DYNAMIC_BODY;

	bd.m_position = m_app.getTransformById(xfId).position;
	bd.m_orientation = m_app.getTransformById(xfId).orientation;
	bd.m_xfId = xfId;
	bd.m_linearDamping = 0.0001f;
	bd.m_angularDamping = 0.0001f;

	void *bodyMemory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(Rigidbody));
	Rigidbody *body = new (static_cast<Rigidbody *>(bodyMemory)) Rigidbody(&bd, this);

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

	BoxShape *box = shape->clone();
	FixtureDef fd;
	fd.shape = box;
	fd.friction = 0.6f;
	fd.restitution = 0.4f;

	body->createFixture(&fd);
	if (m_rigidbodyCount != 0)
	{
		m_rigidbodies->prev = body;
	}

	body->prev = nullptr;
	body->next = m_rigidbodies;
	m_rigidbodies = body;

	++m_rigidbodyCount;
}

void World::createSphere(std::unique_ptr<Model> &model, int32_t xfId)
{
	// std::cout << "World::Create Sphere\n";
	Shape *s = model->getShape();
	SphereShape *shape = dynamic_cast<SphereShape *>(s);

	BodyDef bd;
	// set sphere definition
	bd.m_type = EBodyType::DYNAMIC_BODY;
	bd.m_position = m_app.getTransformById(xfId).position;
	bd.m_orientation = m_app.getTransformById(xfId).orientation;
	bd.m_xfId = xfId;
	bd.m_linearDamping = 0.0001f;
	bd.m_angularDamping = 0.0001f;

	void *bodyMemory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(Rigidbody));
	Rigidbody *body = new (static_cast<Rigidbody *>(bodyMemory)) Rigidbody(&bd, this);

	// calculate inersiaTensor - (2 / 5) * m * r * r
	float mass = 10.0f;
	float r = shape->m_radius;
	float val = (2.0f / 5.0f) * mass * r * r;
	glm::mat3 m(glm::vec3(val, 0.0f, 0.0f), glm::vec3(0.0f, val, 0.0f), glm::vec3(0.0f, 0.0f, val));
	body->setMassData(mass, m);

	SphereShape *sphere = shape->clone();

	FixtureDef fd;
	fd.shape = sphere;
	fd.friction = 0.2f;
	fd.restitution = 0.8f;
	body->createFixture(&fd);

	if (m_rigidbodyCount != 0)
	{
		m_rigidbodies->prev = body;
	}

	body->prev = nullptr;
	body->next = m_rigidbodies;
	m_rigidbodies = body;

	++m_rigidbodyCount;
	// std::cout << "World:: Create Sphere end\n";
}

void World::createGround(std::unique_ptr<Model> &model, int32_t xfId)
{
	// std::cout << "World::Create Box\n";
	Shape *s = model->getShape();
	BoxShape *shape = dynamic_cast<BoxShape *>(s);
	BodyDef bd;

	bd.m_position = m_app.getTransformById(xfId).position;
	bd.m_orientation = m_app.getTransformById(xfId).orientation;
	bd.m_xfId = xfId;
	bd.m_linearDamping = 0.0f;
	bd.m_angularDamping = 0.0f;

	void *bodyMemory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(Rigidbody));
	Rigidbody *body = new (static_cast<Rigidbody *>(bodyMemory)) Rigidbody(&bd, this);

	// calculate inersiaTensor
	glm::mat3 m(0.0f);

	float mass = 0;
	body->setMassData(mass, m);

	BoxShape *box = shape->clone();
	FixtureDef fd;
	fd.shape = box;
	fd.friction = 0.7f;
	fd.restitution = 0.3f;

	body->createFixture(&fd);
	if (m_rigidbodyCount != 0)
	{
		m_rigidbodies->prev = body;
	}

	body->prev = nullptr;
	body->next = m_rigidbodies;
	m_rigidbodies = body;

	++m_rigidbodyCount;
}

void World::createCylinder(std::unique_ptr<Model> &model, int32_t xfId)
{
	Shape *s = model->getShape();
	CylinderShape *shape = dynamic_cast<CylinderShape *>(s);
	BodyDef bd;

	bd.m_type = EBodyType::DYNAMIC_BODY;

	bd.m_position = m_app.getTransformById(xfId).position;
	bd.m_orientation = m_app.getTransformById(xfId).orientation;
	bd.m_xfId = xfId;
	bd.m_linearDamping = 0.0001f;
	bd.m_angularDamping = 0.0001f;

	void *bodyMemory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(Rigidbody));
	Rigidbody *body = new (static_cast<Rigidbody *>(bodyMemory)) Rigidbody(&bd, this);

	// calculate inersiaTensor
	float mass = 30.0f;
	float r = shape->m_radius;
	float h = shape->m_height;
	float Ixx = (1.0f / 12.0f) * (3.0f * r * r + h * h) * mass;
	float Iyy = Ixx;
	float Izz = (1.0f / 2.0f) * (r * r) * mass;
	glm::mat3 m(glm::vec3(Ixx, 0.0f, 0.0f), glm::vec3(0.0f, Iyy, 0.0f), glm::vec3(0.0f, 0.0f, Izz));

	body->setMassData(mass, m);

	CylinderShape *cylinder = shape->clone();
	FixtureDef fd;
	fd.shape = cylinder;
	fd.friction = 0.6f;
	fd.restitution = 0.4f;

	body->createFixture(&fd);

	if (m_rigidbodyCount != 0)
	{
		m_rigidbodies->prev = body;
	}

	body->prev = nullptr;
	body->next = m_rigidbodies;
	m_rigidbodies = body;

	++m_rigidbodyCount;
}

void World::createCapsule(std::unique_ptr<Model> &model, int32_t xfId)
{
	Shape *s = model->getShape();
	CapsuleShape *shape = dynamic_cast<CapsuleShape *>(s);
	BodyDef bd;

	bd.m_type = EBodyType::DYNAMIC_BODY;

	bd.m_position = m_app.getTransformById(xfId).position;
	bd.m_orientation = m_app.getTransformById(xfId).orientation;
	bd.m_xfId = xfId;
	bd.m_linearDamping = 0.0001f;
	bd.m_angularDamping = 0.0001f;

	void *bodyMemory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(Rigidbody));
	Rigidbody *body = new (static_cast<Rigidbody *>(bodyMemory)) Rigidbody(&bd, this);

	// hemisphere
	float mh = 3.0f;
	float r = shape->m_radius;
	float h = shape->m_height;
	float d = (3.0f * r / 8.0f);
	float val = (2.0f / 5.0f) * mh * r * r + (h / 2.0f * d * d);
	glm::mat3 ih(glm::vec3(val, 0.0f, 0.0f), glm::vec3(0.0f, val, 0.0f), glm::vec3(0.0f, 0.0f, val));

	// cylinder
	float mc = 10.0f;
	float Ixx = (1.0f / 12.0f) * (3.0f * r * r + h * h) * mc;
	float Iyy = Ixx;
	float Izz = (1.0f / 2.0f) * (r * r) * mc;
	glm::mat3 ic(glm::vec3(Ixx, 0.0f, 0.0f), glm::vec3(0.0f, Iyy, 0.0f), glm::vec3(0.0f, 0.0f, Izz));

	float mass = mh * 2.0f + mc;
	glm::mat3 m = ih * 2.0f + ic;

	body->setMassData(mass, m);

	CapsuleShape *capsule = shape->clone();
	FixtureDef fd;
	fd.shape = capsule;
	fd.friction = 0.6f;
	fd.restitution = 0.4f;

	body->createFixture(&fd);

	if (m_rigidbodyCount != 0)
	{
		m_rigidbodies->prev = body;
	}

	body->prev = nullptr;
	body->next = m_rigidbodies;
	m_rigidbodies = body;

	++m_rigidbodyCount;
}

void World::registerBodyForce(int32_t idx, const glm::vec3 &force)
{
	// check idx
	Rigidbody *body = m_rigidbodies;
	for (int32_t i = m_rigidbodyCount - 1; i > idx; --i)
	{
		body = body->next;
	}
	body->registerForce(force);
}

} // namespace ale