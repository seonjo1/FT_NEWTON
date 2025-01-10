#include "physics/BoxToCapsuleContact.h"

namespace ale
{

BoxToCapsuleContact::BoxToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *BoxToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new BoxToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 BoxToCapsuleContact::supportA(const ConvexInfo &box, glm::vec3 dir)
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

glm::vec3 BoxToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
{
}

void BoxToCapsuleContact::findCollisionPoints(const ConvexInfo &box, const ConvexInfo &capsule,
											  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
											  std::vector<Simplex> &simplexVector)
{
}

} // namespace ale