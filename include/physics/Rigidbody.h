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

enum class EBodyType
{
	STATIC_BODY = 0,
	KINEMATIC_BODY,
	DYNAMIC_BODY
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
		m_position = glm::vec3(0.0f);
		// m_angle = 0.0f;
		m_orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		m_linearVelocity = glm::vec3(0.0f);
		m_angularVelocity = glm::vec3(0.0f);
		m_linearDamping = 0.0f;
		m_angularDamping = 0.0f;
		// m_canSleep = true;
		// m_isAwake = true;
		m_type = EBodyType::STATIC_BODY;
		m_gravityScale = 15.0f;
	}

	EBodyType m_type;
	glm::vec3 m_position;
	glm::quat m_orientation;
	// float m_angle;
	glm::vec3 m_linearVelocity;
	glm::vec3 m_angularVelocity;
	float m_linearDamping;
	float m_angularDamping;
	bool m_canSleep;
	bool m_isAwake;
	// void *userData;
	float m_gravityScale;
	int32_t m_xfId;
};

class Rigidbody
{
  public:
	Rigidbody(const BodyDef *bd, World *world);
	~Rigidbody();

	void scale(float scale);
	void translate(float distance);
	void addForce(const glm::vec3 &force);
	void addTorque(const glm::vec3 &torque);
	void addGravity();
	void integrate(float duration);
	void updateSweep();
	void registerForce(const glm::vec3 &force);
	void createFixture(Shape *shape);
	void createFixture(const FixtureDef *fd);
	void addForceAtPoint(const glm::vec3 &force, const glm::vec3 &point);
	void addForceAtBodyPoint(const glm::vec3 &force, const glm::vec3 &point);
	void clearAccumulators();
	void synchronizeFixtures();
	void calculateDerivedData();
	void calculateForceAccum();

	bool hasFlag(EBodyFlag flag);
	bool shouldCollide(const Rigidbody *other) const;

	// getter function
	float getInverseMass() const;
	int32_t getTransformId() const;
	int32_t getIslandIndex() const;
	int32_t getBodyId() const;
	EBodyType getType() const;
	ContactLink *getContactLinks();
	glm::vec3 getPointInWorldSpace(const glm::vec3 &point) const;
	const glm::vec3 &getPosition() const;
	const glm::quat &getOrientation() const;
	const Transform &getTransform() const;
	const glm::mat4 &getTransformMatrix() const;
	const glm::vec3 &getLinearVelocity() const;
	const glm::vec3 &getAngularVelocity() const;
	const glm::vec3 &getAcceleration() const;
	const glm::mat3 &getInverseInertiaTensorWorld() const;

	void setFlag(EBodyFlag flag);
	void unsetFlag(EBodyFlag flag);
	void setMassData(float mass, const glm::mat3 &inertiaTensor);
	void setPosition(const glm::vec3 &position);
	void setIslandIndex(int32_t idx);
	void setOrientation(const glm::quat &orientation);
	void setAcceleration(const glm::vec3 &acceleration);
	void setContactLinks(ContactLink *contactLink);
	void setLinearVelocity(const glm::vec3 &linearVelocity);
	void setAngularVelocity(const glm::vec3 &angularVelocity);

  protected:
	static int32_t BODY_COUNT;

	World *m_world;

	Sweep m_sweep;
	Transform m_xf;
	glm::vec3 m_linearVelocity;
	glm::vec3 m_angularVelocity;
	glm::mat3 m_inverseInertiaTensorWorld;
	glm::mat3 m_inverseInertiaTensor;
	glm::mat4 m_transformMatrix;
	glm::vec3 m_forceAccum;
	glm::vec3 m_torqueAccum;
	glm::vec3 m_acceleration;
	glm::vec3 m_lastFrameAcceleration;
	std::vector<Fixture *> m_fixtures;
	std::queue<glm::vec3> m_forceRegistry;

	// float motion;
	// bool isAwake;
	// bool canSleep;

	float m_inverseMass;
	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;
	int32_t m_xfId;
	int32_t m_flags;
	int32_t m_islandIndex;
	int32_t m_bodyID;
	EBodyType m_type;

	ContactLink *m_contactLinks;

  private:
};
} // namespace ale

#endif