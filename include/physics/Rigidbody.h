#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "common.h"

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
	BodyDef()
	{
		// userData = nullptr;
		// set position
		// position()
		angle = 0.0f;
		linearVelocity = 0.0f;
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
	float gravityScale;
};

class Rigidbody
{
  public:
	Rigidbody(const BodyDef *bd);
	void integrate(float duration);
	void calculateDerivedData();
	void addForce(const glm::vec3 &force);
	void addForceAtPoint(const glm::vec3 &force, const glm::vec3 &point);
	void addForceAtBodyPoint(const glm::vec3 &force, const glm::vec3 &point);
	void addTorque(const glm::vec3 &torque);
	void clearAccumulators();

	void translate(float distance);
	void scale(float scale);

	glm::vec3 getPointInWorldSpace(const glm::vec3 &point) const;

	// getter function
	const glm::vec3 &getPosition() const;
	const glm::vec3 &getOrientation() const;
	const glm::vec3 &getLinearVelocity() const;
	const glm::vec3 &getAngularVelocity() const;
	const glm::vec3 &getAcceleration() const;

	void setPosition(const glm::vec3 &position);
	void setOrientation(const glm::quat &orientation);
	void setLinearVelocity(const glm::vec3 &linearVelocity);
	void setAngularVelocity(const glm::vec3 &angularVelocity);
	void setAcceleration(const glm::vec3 &acceleration);

  protected:
	glm::vec3 position;
	glm::quat orientation;
	glm::vec3 linearVelocity;
	glm::vec3 angularVelocity;
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

	float inverseMass;
	float linearDamping;
	float angularDamping;
	float gravityScale;

  private:
};
} // namespace ale
#endif