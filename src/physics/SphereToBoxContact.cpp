#include "physics/SphereToBoxContact.h"

namespace ale
{

SphereToBoxContact::SphereToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToBoxContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToBoxContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 SphereToBoxContact::supportA(const ConvexInfo &sphere, glm::vec3 dir)
{
	return sphere.center + dir * sphere.radius;
}

glm::vec3 SphereToBoxContact::supportB(const ConvexInfo &box, glm::vec3 dir)
{
	float dotAxes[3] = {glm::dot(box.axes[0], dir) > 0 ? 1.0f : -1.0f, glm::dot(box.axes[1], dir) > 0 ? 1.0f : -1.0f,
						glm::dot(box.axes[2], dir) > 0 ? 1.0f : -1.0f};

	glm::vec3 point = box.center;
	for (int i = 0; i < 3; ++i)
	{
		point += box.axes[i] * (dotAxes[i] * box.halfSize[i]);
	}

	return point;
}

void SphereToBoxContact::findCollisionPoints(const ConvexInfo &sphere, const ConvexInfo &box,
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