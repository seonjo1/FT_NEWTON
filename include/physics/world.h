#ifndef WORLD_H
#define WORLD_H

#include "Common.h"
#include "ContactManager.h"
#include "Island.h"
#include <stack>

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
	World(App &app);
	~World();

	void startFrame();
	void runPhysics(float duration);
	void solve(float duration);
	void createBody(std::unique_ptr<Model> &model, int32_t xfId);
	void createBox(std::unique_ptr<Model> &model, int32_t xfId);
	void createSphere(std::unique_ptr<Model> &model, int32_t xfId);
	void createGround(std::unique_ptr<Model> &model, int32_t xfId);
	void createCylinder(std::unique_ptr<Model> &model, int32_t xfId);
	void createCapsule(std::unique_ptr<Model> &model, int32_t xfId);
	void registerBodyForce(int32_t idx, const glm::vec3 &force);

	ContactManager m_contactManager;
	App &m_app;

  private:
	Rigidbody *m_rigidbodies;
	int32_t m_rigidbodyCount;
};
} // namespace ale
#endif