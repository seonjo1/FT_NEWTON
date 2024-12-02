#include "world.h"
#include "BoxShape.h"
#include "Rigidbody.h"
#include "model.h"

namespace ale
{
World::World(uint32_t size)
{
}

void World::startFrame()
{
	for (std::unique_ptr<Rigidbody> &body : rigidbodies)
	{
		body->clearAccumulators();
		body->calculateDerivedData();
	}
}

void World::runPhysics()
{
	for (std::unique_ptr<Rigidbody> &body : rigidbodies)
	{
		body->integrate(0.1);

		// update Dynamic Tree if Body moved more than fat AABB
		// update Possible Contact Pairs - BroadPhase
	}
	// Process Contacts
}

void World::createBody(std::unique_ptr<Model> &model)
{
	std::unique_ptr<Shape> shape = model->getShape();
	Type type = shape->getType();

	switch (type)
	{
	case Type::e_box:
		createBox(shape);
		break;
	case Type::e_sphere:
		createSphere(shape);
		break;
	default:
		break;
	}
}

void World::createBox(std::unique_ptr<Shape> &shape)
{
	BodyDef bd;

	// set box definition
	bd.type = BodyType::e_dynamic;
	bd.position;
	bd.linearDamping = 0.01f;
	bd.angularDamping = 0.01f;

	std::unique_ptr<Rigidbody> body = new Rigidbody(&bd, this);

	// create fixture of body - shape needs vertex info
	std::unique_ptr<Shape> box = shape->clone();
	body->createFixture(box);
	rigidbodies.push_back(body);
	// set model->body
}

void World::createSphere(std::unique_ptr<Shape> &shape)
{
	BodyDef bd;
	// set sphere definition
	bd.type = BodyType::e_dynamic;
	bd.position;
	bd.linearDamping = 0.01f;
	bd.angularDamping = 0.01f;

	std::unique_ptr<Rigidbody> body = new Rigidbody(&bd);

	// create fixture of body
	std::unique_ptr<Shape> sphere = shape->clone();
	body->createFixture(sphere);
	rigidbodies.push_back(body);
	// set model->body
}
} // namespace ale