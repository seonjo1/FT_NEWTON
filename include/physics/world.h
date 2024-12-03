#ifndef WORLD_H
#define WORLD_H

#include "ContactManager.h"
#include "common.h"

class Model;

namespace ale
{
class Rigidbody;
class ContactManager;
class BoxShape;
class SphereShape;

class World
{
  public:
	World(uint32_t size);
	void startFrame();
	void runPhysics();
	void createBody(std::unique_ptr<Model> &model);
	void createBox(std::unique_ptr<Model> &model);
	void createSphere(std::unique_ptr<Model> &model);

	ContactManager contactManager;

  private:
	std::vector<Rigidbody *> rigidbodies;
};
} // namespace ale
#endif