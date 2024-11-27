#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace ale
{

class Fixture;

enum class BodyType
{
	e_static = 0,
	e_kinematic,
	e_dynamic
};

struct BodyDef
{
	b2BodyDef()
	{
		// userData = nullptr;
		// set position
		// position()
		angle = 0.0f;
		// set linearVelocity
		angularVelocity = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		canSleep = true;
		isAwake = true;
		type = e_static;
	}

	BodyType type;
	glm::vec3 position;
	float angle;
	glm::vec3 linearVelocity;
	glm::vec3 angularVelocity;
	float linearDamping;
	float angularDamping;
	bool canSleep;
	bool isAwake;
	// void *userData;
	// float gravityScale;
};

class Rigidbody
{
  public:
	Rigidbody(const BodyDef *bd);
	void integrate();
	void calculateDerivedData();
	void addForce();
	void addForceAtPoint();
	void addForceAtBodyPoint();
	void addTorque();
	void clearAccumulators();

  protected:
	glm::vec3 position;
	glm::quat orientation;
	glm::vec3 velocity;
	glm::vec3 rotation;
	glm::mat3 inverseInertiaTensorWorld;
	float motion;
	bool isAwake;
	bool canSleep;
	glm::mat4 transformMatrix;
	glm::vec3 forceAccum;
	glm::vec3 torqueAccum;
	glm::vec3 acceleration;
	glm::vec3 lastFrameAcceleration;
	// std::vector<Fixture*> fixtures;

  private:
};
} // namespace ale
#endif