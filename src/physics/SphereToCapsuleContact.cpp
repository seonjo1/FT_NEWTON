#include "physics/SphereToCapsuleContact.h"

namespace ale
{

SphereToCapsuleContact::SphereToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 SphereToCapsuleContact::supportA(const ConvexInfo &sphere, glm::vec3 dir)
{
	return sphere.center + dir * sphere.radius;
}

glm::vec3 SphereToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
{
	float dotResult = glm::dot(dir, capsule.axes[0]);
	if (dotResult > 0)
	{
		return capsule.points[0] + dir * capsule.radius;
	}
	else
	{
		return capsule.points[1] + dir * capsule.radius;
	}
}

void SphereToCapsuleContact::findCollisionPoints(const ConvexInfo &sphere, const ConvexInfo &capsule,
												  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
												  std::vector<Simplex> &simplexVector)
{
	CollisionInfo collisionInfo;

	collisionInfo.normal = epaInfo.normal;
	collisionInfo.seperation = epaInfo.distance;
	collisionInfo.pointA = sphere.center + epaInfo.normal * sphere.radius;
	collisionInfo.pointB = collisionInfo.pointA - collisionInfo.normal * collisionInfo.seperation;
	
	collisionInfoVector.push_back(collisionInfo);
}

} // namespace ale