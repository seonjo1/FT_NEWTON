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

World::~World()
{
	for (Rigidbody *body : rigidbodies)
	{
		delete body;
	}
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
	// contactManager.findNewContacts();
	// Process Contacts - NarrowPhase
}

void World::createBody(std::unique_ptr<Model> &model, int32_t xfId)
{
	std::cout << "World::Create Body\n";
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
	glm::vec3 upper = *std::prev(shape->m_vertices.end());
	glm::vec3 lower = *shape->m_vertices.begin();
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

void World::createGround(std::unique_ptr<Model> &model, int32_t xfId)
{
	std::cout << "World::Create Box\n";
	Shape *s = model->getShape();
	BoxShape *shape = dynamic_cast<BoxShape *>(s);
	BodyDef bd;

	// set box definition
	bd.type = BodyType::e_static;

	bd.position = app.getTransformById(xfId).position;
	bd.orientation = app.getTransformById(xfId).orientation;
	bd.xfId = xfId;
	bd.linearDamping = 0.01f;
	bd.angularDamping = 0.01f;

	Rigidbody *body = new Rigidbody(&bd, this);

	// calculate inersiaTensor
	glm::vec3 upper = *std::prev(shape->m_vertices.end());
	glm::vec3 lower = *shape->m_vertices.begin();
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

void World::registerBodyForce(int32_t idx, const glm::vec3 &force)
{
	// check idx
	rigidbodies[idx]->registerForce(force);
}

} // namespace ale