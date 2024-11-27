#include "world.h"

namespace ale
{
World::World()
{
}

void World::startFrame()
{
}

void World::runPhysics()
{
}

void World::createBody(std::unique<Model> &model)
{
	switch (type)
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