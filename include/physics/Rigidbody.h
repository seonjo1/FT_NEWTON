#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "physics/BoxShape.h"
#include "physics/SphereShape.h"
#include <queue>

namespace ale
{
class World;
class Fixture;
struct FixtureDef;
struct ContactLink;

enum class BodyType
{
	e_static = 0,
	e_kinematic,
	e_dynamic
};

enum class EBodyFlag
{
	ISLAND = (1 << 0),
};

struct BodyDef
{
	BodyDef()
	{
		// userData = nullptr;
		// set position
		// position()
		position = glm::vec3(0.0f);
		angle = 0.0f;
		orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		linearVelocity = glm::vec3(0.0f);
		angularVelocity = glm::vec3(0.0f);
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		canSleep = true;
		isAwake = true;
		type = BodyType::e_static;
		gravityScale = 1.0f;
	}

	BodyType type;
	glm::vec3 position;
	glm::quat orientation;
	float angle;
	glm::vec3 linearVelocity;
	glm::vec3 angularVelocity;
	float linearDamping;
	float angularDamping;
	bool canSleep;
	bool isAwake;
	// void *userData;
	float gravityScale;
	int32_t xfId;
};

class Rigidbody
{
  public:
	Rigidbody(const BodyDef *bd, World *world);

	void synchronizeFixtures();
	void synchronizeTransform();
	void integrate(float duration);
	void calculateDerivedData();
	void addForce(const glm::vec3 &force);
	void addForceAtPoint(const glm::vec3 &force, const glm::vec3 &point);
	void addForceAtBodyPoint(const glm::vec3 &force, const glm::vec3 &point);
	void addTorque(const glm::vec3 &torque);
	void addGravity();
	void registerForce(const glm::vec3 &force);
	void calculateForceAccum();
	void clearAccumulators();

	void translate(float distance);
	void scale(float scale);

	glm::vec3 getPointInWorldSpace(const glm::vec3 &point) const;

	// getter function
	const glm::vec3 &getPosition() const;
	const glm::quat &getOrientation() const;
	const Transform &getTransform() const;
	const glm::mat4 &getTransformMatrix() const;
	const glm::vec3 &getLinearVelocity() const;
	const glm::vec3 &getAngularVelocity() const;
	const glm::vec3 &getAcceleration() const;
	const glm::mat3 &getInverseInertiaTensorWorld() const;
	float getInverseMass() const;
	BodyType getType();
	int32_t getTransformId() const;
	int32_t getIslandIndex() const;
	ContactLink *getContactLinks();
	int32_t getBodyId() const;

	void setPosition(const glm::vec3 &position);
	void setOrientation(const glm::quat &orientation);
	void setLinearVelocity(const glm::vec3 &linearVelocity);
	void setAngularVelocity(const glm::vec3 &angularVelocity);
	void setAcceleration(const glm::vec3 &acceleration);
	void setMassData(float mass, const glm::mat3 &inertiaTensor);
	void setContactLinks(ContactLink *contactLink);
	void setIslandIndex(int32_t idx);
	void setFlag(EBodyFlag flag);
	void unsetFlag(EBodyFlag flag);

	void createFixture(Shape *shape);
	void createFixture(const FixtureDef *fd);

	bool shouldCollide(const Rigidbody *other) const;
	bool hasFlag(EBodyFlag flag);
	void updateSweep();
	
	static glm::vec3 gravity;

  protected:
	static int32_t BODY_COUNT;

	World *world;
	Transform xf;
	glm::vec3 linearVelocity;
	glm::vec3 angularVelocity;
	glm::mat3 inverseInertiaTensorWorld;
	glm::mat3 inverseInertiaTensor;
	glm::mat4 transformMatrix;
	glm::vec3 forceAccum;
	glm::vec3 torqueAccum;
	glm::vec3 acceleration;
	glm::vec3 lastFrameAcceleration;
	std::vector<Fixture *> fixtures;
	std::queue<glm::vec3> forceRegistry;
	Sweep sweep;

	float motion;
	bool isAwake;
	bool canSleep;

	BodyType type;
	float inverseMass;
	float linearDamping;
	float angularDamping;
	float gravityScale;
	int32_t xfId;
	int32_t m_flags;
	int32_t m_islandIndex;
	int32_t m_bodyID;

	ContactLink *m_contactLinks;

  private:
};
} // namespace ale

#endif