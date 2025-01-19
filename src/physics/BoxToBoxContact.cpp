#include "physics/BoxToBoxContact.h"

namespace ale
{

BoxToBoxContact::BoxToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *BoxToBoxContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(BoxToBoxContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<BoxToBoxContact *>(memory)) BoxToBoxContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 BoxToBoxContact::supportA(const ConvexInfo &box, glm::vec3 dir)
{
	float dotAxes[3] = {glm::dot(box.axes[0], dir) > 0 ? 1.0f : -1.0f, glm::dot(box.axes[1], dir) > 0 ? 1.0f : -1.0f,
						glm::dot(box.axes[2], dir) > 0 ? 1.0f : -1.0f};

	glm::vec3 point = box.center;
	for (int32_t i = 0; i < 3; ++i)
	{
		point += box.axes[i] * (dotAxes[i] * box.halfSize[i]);
	}

	return point;
}

glm::vec3 BoxToBoxContact::supportB(const ConvexInfo &box, glm::vec3 dir)
{
	float dotAxes[3] = {glm::dot(box.axes[0], dir) > 0 ? 1.0f : -1.0f, glm::dot(box.axes[1], dir) > 0 ? 1.0f : -1.0f,
						glm::dot(box.axes[2], dir) > 0 ? 1.0f : -1.0f};

	glm::vec3 point = box.center;
	for (int32_t i = 0; i < 3; ++i)
	{
		point += box.axes[i] * (dotAxes[i] * box.halfSize[i]);
	}

	return point;
}

void BoxToBoxContact::findCollisionPoints(const ConvexInfo &boxA, const ConvexInfo &boxB,
										  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
										  std::vector<Simplex> &simplexVector)
{
	// clipping
	Face refFace = getBoxFace(boxA, epaInfo.normal);
	Face incFace = getBoxFace(boxB, -epaInfo.normal);

	std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);

	// 폴리곤의 각 꼭지점 -> 충돌점 여러 개
	buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
}

} // namespace ale