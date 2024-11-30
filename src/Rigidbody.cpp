#include "Rigidbody.h"

namespace ale
{
Rigidbody::Rigidbody(const BodyDef *bd)
{
	type = bd->type;

	// Transform struct needed
	position = bd->position;
	orientation = bd->angle;

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
}

} // namespace ale