#ifndef WORLD_H
#define WORLD_H

#include "Rigidbody.h"
#include "common.h"
#include "model.h"

namespace ale
{
class Rigidbody;

class World
{
  public:
	World(uint32_t size);
	void startFrame();
	void runPhysics();
	void createBody(std::unique_ptr<Model> &model);
	void createBox();
	void createSphere();

  private:
	std::vector<std::unique_ptr<Rigidbody>> rigidbodies;
};
} // namespace ale
#endif