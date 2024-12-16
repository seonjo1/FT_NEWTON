#ifndef WORLD_H
#define WORLD_H

#include "ContactManager.h"
#include "common.h"

class Model;
class App;

namespace ale
{
class Rigidbody;
class ContactManager;
class BoxShape;
class SphereShape;

class World
{
  public:
	World(uint32_t size, App &app);
	void startFrame();
	void runPhysics();
	void createBody(std::unique_ptr<Model> &model, int32_t xfId);
	void createBox(std::unique_ptr<Model> &model, int32_t xfId);
	void createSphere(std::unique_ptr<Model> &model, int32_t xfId);
	void createGround(std::unique_ptr<Model> &model, int32_t xfId);
	void registerBodyForce(int32_t idx, const glm::vec3 &force);

	ContactManager contactManager;
	App &app;

  private:
	std::vector<Rigidbody *> rigidbodies;
};
} // namespace ale
#endif