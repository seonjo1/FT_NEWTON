#include "Rigidbody.h"
#include "Fixture.h"
#include "Shape.h"
#include "world.h"

namespace ale
{
static inline glm::mat4 _calculateTransformMatrix(glm::mat4 &transformMatrix, const glm::vec3 &position,
												  const glm::quat &orientation)
{
	glm::mat4 rotationMatrix = glm::toMat4(orientation);

	glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);

	transformMatrix = translationMatrix * rotationMatrix;
}

static inline void _transformInertiaTensor(glm::mat3 &iitWorld, const glm::quat &q, const glm::mat3 &iitBody,
										   const glm::mat4 &rotmat)
{
	glm::mat3 rotationMatrix = glm::mat3(rotmat);

	iitWorld = rotationMatrix * iitBody * glm::transpose(rotationMatrix);
}

Rigidbody::Rigidbody(const BodyDef *bd, World *world)
{
	this->world = world;
	type = bd->type;

	// Transform struct needed
	xf.Set(bd->position, bd->angle);

	linearVelocity = bd->linearVelocity;
	angularVelocity = bd->angularVelocity;

	linearDamping = bd->linearDamping;
	angularDamping = bd->angularDamping;
	gravityScale = bd->gravityScale;

	canSleep = bd->canSleep;
	isAwake = bd->isAwake;
}

// Update acceleration by Adding force to Body
void Rigidbody::integrate(float duration)
{
	// Set acceleration by F = ma
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration += (forceAccum * inverseMass);

	// set angular acceleration
	glm::vec3 angularAcceleration = inverseInertiaTensorWorld * torqueAccum;

	// set velocity by accerleration
	linearVelocity += (lastFrameAcceleration * duration);
	angularVelocity += (angularAcceleration * duration);

	// impose drag
	/// linearDamping
	// angularDamping

	// set position
	position += (linearVelocity * duration);
	orientation += (angularVelocity * duration);

	calculateDerivedData();
	clearAccumulators();

	// if can sleep ~
}

void Rigidbody::calculateDerivedData()
{
	glm::quat q = glm::normalize(xf.orientation);

	_calculateTransformMatrix(transformMatrix, xf.position, q);
	_transformInertiaTensor(inverseInertiaTensorWorld, q, inverseInertiaTensor, transformMatrix);
}

void Rigidbody::addForce(const glm::vec3 &force)
{
	forceAccum += force;
}

void Rigidbody::addForceAtPoint(const glm::vec3 &force, const glm::vec3 &point)
{
	glm::vec3 pt = point;
	pt -= position;

	forceAccum += force;
	torqueAccum += glm::cross(pt, force);
}

void Rigidbody::addForceAtBodyPoint(const glm::vec3 &force, const glm::vec3 &point)
{
	glm::vec3 pt = getPointInWorldSpace(point);
	addForceAtPoint(pt);
}

void Rigidbody::addTorque(const glm::vec3 &torque)
{
	torqueAccum += torque;
}

void Rigidbody::clearAccumulators()
{
	// clear accumulate vector to zero
	forceAccum.x = 0;
	forceAccum.y = 0;
	forceAccum.z = 0;

	torqueAccum.x = 0;
	torqueAccum.y = 0;
	torqueAccum.z = 0;
}

glm::vec3 Rigidbody::getPointInWorldSpace(const glm::vec3 &point)
{
	return (transformMatrix * point);
}

void Rigidbody::createFixture(const std::unique_ptr<Shape> &shape)
{
	FixtureDef fd;
	fd.shape = shape;

	createFixture(&fd);
}

void Rigidbody::createFixture(const FixtureDef *fd)
{
	std::unique_ptr<Fixture> fixture = new Fixture();

	fixture->Create(&fd);
	fixture->CreateProxies(&world->contactManager.broadPhase);
	fixtures.push_back(fixture);
}

} // namespace ale