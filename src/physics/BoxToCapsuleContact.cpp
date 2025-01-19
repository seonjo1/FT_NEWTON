#include "physics/BoxToCapsuleContact.h"

namespace ale
{

BoxToCapsuleContact::BoxToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *BoxToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(BoxToCapsuleContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<BoxToCapsuleContact *>(memory)) BoxToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 BoxToCapsuleContact::supportA(const ConvexInfo &box, glm::vec3 dir)
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

glm::vec3 BoxToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
{
	float dotResult = glm::dot(dir, capsule.axes[0]);

	glm::vec3 move(0.0f);

	if (dotResult > 0)
	{
		move = capsule.axes[0] * capsule.height * 0.5f;
	}
	else if (dotResult < 0)
	{
		move = -capsule.axes[0] * capsule.height * 0.5f;
	}

	return capsule.center + move + dir * capsule.radius;
}

void BoxToCapsuleContact::findCollisionPoints(const ConvexInfo &box, const ConvexInfo &capsule,
											  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
											  std::vector<Simplex> &simplexVector)
{
	// std::cout << "box to capsule!!!!\n";
	if (isCollideToHemisphere(capsule, -epaInfo.normal))
	{
		CollisionInfo collisionInfo;

		collisionInfo.normal = epaInfo.normal;
		collisionInfo.seperation = epaInfo.distance;

		if (glm::dot(capsule.axes[0], collisionInfo.normal) < 0)
		{
			glm::vec3 hemisphereCenter = capsule.center + capsule.axes[0] * 0.5f * capsule.height;
			collisionInfo.pointB = hemisphereCenter - collisionInfo.normal * capsule.radius;
			collisionInfo.pointA = collisionInfo.pointB + collisionInfo.normal * collisionInfo.seperation;
		}
		else
		{
			glm::vec3 hemisphereCenter = capsule.center - capsule.axes[0] * 0.5f * capsule.height;
			collisionInfo.pointB = hemisphereCenter - collisionInfo.normal * capsule.radius;
			collisionInfo.pointA = collisionInfo.pointB + collisionInfo.normal * collisionInfo.seperation;
		}

		collisionInfoVector.push_back(collisionInfo);
	}
	else
	{
		// std::cout << "edge side!!!\n";
		Face refFace = getBoxFace(box, epaInfo.normal);
		Face incFace = getCapsuleFace(capsule, -epaInfo.normal);

		std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);
		buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
	}
}

} // namespace ale