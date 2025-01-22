#include "physics/CapsuleToCapsuleContact.h"

namespace ale
{

CapsuleToCapsuleContact::CapsuleToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *CapsuleToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(CapsuleToCapsuleContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<CapsuleToCapsuleContact *>(memory))
		CapsuleToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 CapsuleToCapsuleContact::supportA(const ConvexInfo &capsule, glm::vec3 dir)
{
	// std::cout << "supportA dir: " << dir.x << " " << dir.y << " " << dir.z << "\n";
	float dotResult = glm::dot(dir, capsule.axes[0]);

	glm::vec3 move(0.0f);

	// std::cout << "dotResult: " << dotResult << "\n";
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

glm::vec3 CapsuleToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
{
	// std::cout << "supportB dir: " << dir.x << " " << dir.y << " " << dir.z << "\n";

	float dotResult = glm::dot(dir, capsule.axes[0]);

	glm::vec3 move(0.0f);
	// std::cout << "dotResult: " << dotResult << "\n";

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

void CapsuleToCapsuleContact::findCollisionPoints(const ConvexInfo &capsuleA, const ConvexInfo &capsuleB,
												  CollisionInfo &collisionInfo, EpaInfo &epaInfo,
												  SimplexArray &simplexArray)
{
	// clipping
	if (isCollideToHemisphere(capsuleA, epaInfo.normal))
	{
		collisionInfo.normal[0] = epaInfo.normal;
		collisionInfo.seperation[0] = epaInfo.distance;

		if (glm::dot(capsuleA.axes[0], epaInfo.normal) > 0)
		{
			glm::vec3 hemisphereCenter = capsuleA.center + capsuleA.axes[0] * 0.5f * capsuleA.height;
			collisionInfo.pointA[0] = hemisphereCenter + epaInfo.normal * capsuleA.radius;
			collisionInfo.pointB[0] = collisionInfo.pointA[0] - collisionInfo.normal[0] * collisionInfo.seperation[0];
		}
		else
		{
			glm::vec3 hemisphereCenter = capsuleA.center - capsuleA.axes[0] * 0.5f * capsuleA.height;
			collisionInfo.pointA[0] = hemisphereCenter + epaInfo.normal * capsuleA.radius;
			collisionInfo.pointB[0] = collisionInfo.pointA[0] - collisionInfo.normal[0] * collisionInfo.seperation[0];
		}
		++collisionInfo.size;
	}
	else if (isCollideToHemisphere(capsuleB, -epaInfo.normal))
	{
		collisionInfo.normal[0] = epaInfo.normal;
		collisionInfo.seperation[0] = epaInfo.distance;

		if (glm::dot(capsuleB.axes[0], collisionInfo.normal[0]) < 0)
		{
			glm::vec3 hemisphereCenter = capsuleB.center + capsuleB.axes[0] * 0.5f * capsuleB.height;
			collisionInfo.pointB[0] = hemisphereCenter - collisionInfo.normal[0] * capsuleB.radius;
			collisionInfo.pointA[0] = collisionInfo.pointB[0] + collisionInfo.normal[0] * collisionInfo.seperation[0];
		}
		else
		{
			glm::vec3 hemisphereCenter = capsuleB.center - capsuleB.axes[0] * 0.5f * capsuleB.height;
			collisionInfo.pointB[0] = hemisphereCenter - collisionInfo.normal[0] * capsuleB.radius;
			collisionInfo.pointA[0] = collisionInfo.pointB[0] + collisionInfo.normal[0] * collisionInfo.seperation[0];
		}
		++collisionInfo.size;
	}
	else
	{
		
		Face refFace, incFace;
		
		setCapsuleFace(refFace, capsuleA, epaInfo.normal);
		setCapsuleFace(incFace, capsuleB, -epaInfo.normal);

		ContactPolygon contactPolygon;
		computeContactPolygon(contactPolygon, refFace, incFace);

		buildManifoldFromPolygon(collisionInfo, refFace, incFace, contactPolygon, epaInfo);
	}
}

} // namespace ale