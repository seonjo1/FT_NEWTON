#include "physics/Rigidbody.h"
#include "physics/Fixture.h"
#include "physics/Shape.h"
#include "physics/world.h"

namespace ale
{
static inline void _calculateTransformMatrix(glm::mat4 &transformMatrix, const glm::vec3 &position,
											 const glm::quat &orientation)
{
	glm::mat4 rotationMatrix = glm::toMat4(orientation);

	glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);

	transformMatrix = translationMatrix * rotationMatrix;
}

static inline void _transformInertiaTensor(glm::mat3 &iitWorld, const glm::mat3 &iitBody, const glm::mat4 &rotmat)
{
	glm::mat3 rotationMatrix = glm::mat3(rotmat);

	iitWorld = rotationMatrix * iitBody * glm::transpose(rotationMatrix);
}

Rigidbody::Rigidbody(const BodyDef *bd, World *world)
{
	this->world = world;
	type = bd->type;
	xfId = bd->xfId;

	xf.position = bd->position;
	xf.orientation = bd->orientation;

	linearVelocity = bd->linearVelocity;
	angularVelocity = bd->angularVelocity;

	linearDamping = bd->linearDamping;
	angularDamping = bd->angularDamping;
	gravityScale = bd->gravityScale;

	canSleep = bd->canSleep;
	isAwake = bd->isAwake;
	acceleration = glm::vec3(0.0f);
}

void Rigidbody::synchronizeFixtures()
{
	if (type == BodyType::e_static)
	{
		return;
	}

	Transform xf1;
	xf1.position = sweep.p;
	xf1.orientation = sweep.q;

	BroadPhase *broadPhase = &world->contactManager.broadPhase;
	for (Fixture *fixture : fixtures)
	{
		fixture->synchronize(broadPhase, xf1, xf);
	}
}

// Update acceleration by Adding force to Body
void Rigidbody::integrate(float duration)
{
	if (type == BodyType::e_static)
	{
		return;
	}
	// gravity
	addGravity();

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

	// set sweep (previous Transform)
	sweep.p = xf.position;
	sweep.q = xf.orientation;

	// set position
	xf.position += (linearVelocity * duration);
	// set orientation
	glm::quat angularVelocityQuat = glm::quat(0.0f, angularVelocity * duration); // 각속도를 쿼터니언으로 변환
	xf.orientation += 0.5f * angularVelocityQuat * xf.orientation;				 // 쿼터니언 미분 공식
	xf.orientation = glm::normalize(xf.orientation);							 // 정규화하여 안정성 유지

	calculateDerivedData();
	clearAccumulators();

	// if can sleep ~
}

void Rigidbody::calculateDerivedData()
{
	glm::quat q = glm::normalize(xf.orientation);

	_calculateTransformMatrix(transformMatrix, xf.position, q);
	_transformInertiaTensor(inverseInertiaTensorWorld, inverseInertiaTensor, transformMatrix);
}

void Rigidbody::addForce(const glm::vec3 &force)
{
	forceAccum += force;
}

void Rigidbody::addForceAtPoint(const glm::vec3 &force, const glm::vec3 &point)
{
	glm::vec3 pt = point;
	pt -= xf.position;

	forceAccum += force;
	torqueAccum += glm::cross(pt, force);
}

void Rigidbody::addForceAtBodyPoint(const glm::vec3 &force, const glm::vec3 &point)
{
	glm::vec3 pt = getPointInWorldSpace(point);
	addForceAtPoint(force, pt);
}

void Rigidbody::addTorque(const glm::vec3 &torque)
{
	torqueAccum += torque;
}

void Rigidbody::addGravity()
{
	addForce(glm::vec3(0.0f, -1.0f, 0.0f));
}

void Rigidbody::calculateForceAccum()
{
	while (!forceRegistry.empty())
	{
		addForce(forceRegistry.front());
		forceRegistry.pop();
	}
}

void Rigidbody::registerForce(const glm::vec3 &force)
{
	forceRegistry.push(force);
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

glm::vec3 Rigidbody::getPointInWorldSpace(const glm::vec3 &point) const
{
	glm::vec4 ret = transformMatrix * glm::vec4(point, 1.0f);
	return glm::vec3(ret);
}

const Transform &Rigidbody::getTransform() const
{
	return xf;
}

const glm::vec3 &Rigidbody::getPosition() const
{
	return xf.position;
}

const glm::mat4 &Rigidbody::getTransformMatrix() const
{
	return transformMatrix;
}

int32_t Rigidbody::getTransformId() const
{
	return xfId;
}

void Rigidbody::setPosition(const glm::vec3 &position)
{
	this->xf.position = position;
}

void Rigidbody::setMassData(float mass, const glm::mat3 &inertiaTensor)
{
	// 추후 예외처리
	if (mass == 0)
		return;
	inverseMass = 1 / mass;

	// 역행렬 존재 가능한지 예외처리
	inverseInertiaTensor = inertiaTensor;
	inverseInertiaTensor = glm::inverse(inverseInertiaTensor);
}

void Rigidbody::createFixture(Shape *shape)
{
	std::cout << "Rigidbody::Create Fixture(Shape)\n";
	FixtureDef fd;
	fd.shape = shape;

	createFixture(&fd);
	std::cout << "Rigidbody::Create Fixture(Shape) end\n";
}

void Rigidbody::createFixture(const FixtureDef *fd)
{
	std::cout << "Rigidbody::Create Fixture(FixtureDef)\n";

	Fixture *fixture = new Fixture();

	fixture->Create(this, fd);
	fixture->CreateProxies(&world->contactManager.broadPhase);
	fixtures.push_back(fixture);
	std::cout << "Rigidbody::Create Fixture(FixtureDef) end\n";
}

} // namespace ale