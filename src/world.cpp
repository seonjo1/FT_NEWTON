#include "world.h"

namespace ale
{
World::World()
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
	switch (model->getShapeType())
	{
	case e_box:
		createBox();
		break;
	case e_sphere:
		createSphere();
		break;
	default:
		break;
	}
}

void World::createBox()
{
	BodyDef bd;
	// set box definition
	std::unique_ptr<Rigidbody> body = new Rigidbody(&bd);
	// create fixture of body
	rigidbodies.push_back(body);
	// set model->body
}

void World::createSphere()
{
	BodyDef bd;
	// set sphere definition
	std::unique_ptr<Rigidbody> body = new Rigidbody(&bd);
	// create fixture of body
	rigidbodies.push_back(body);
	// set model->body
}
} // namespace ale