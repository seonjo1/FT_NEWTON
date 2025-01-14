#include "physics/CapsuleToCapsuleContact.h"

namespace ale
{

CapsuleToCapsuleContact::CapsuleToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *CapsuleToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new CapsuleToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
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
												  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
												  std::vector<Simplex> &simplexVector)
{
	// clipping
	if (isCollideToHemisphere(capsuleA, epaInfo.normal))
	{
		CollisionInfo collisionInfo;

		collisionInfo.normal = epaInfo.normal;
		collisionInfo.seperation = epaInfo.distance;
		
		if (glm::dot(capsuleA.axes[0], epaInfo.normal) > 0)
		{
			glm::vec3 hemisphereCenter = capsuleA.center + capsuleA.axes[0] * 0.5f * capsuleA.height;
			collisionInfo.pointA = hemisphereCenter + epaInfo.normal * capsuleA.radius;
			collisionInfo.pointB = collisionInfo.pointA - collisionInfo.normal * collisionInfo.seperation;
		}
		else
		{
			glm::vec3 hemisphereCenter = capsuleA.center - capsuleA.axes[0] * 0.5f * capsuleA.height;
			collisionInfo.pointA = hemisphereCenter + epaInfo.normal * capsuleA.radius;
			collisionInfo.pointB = collisionInfo.pointA - collisionInfo.normal * collisionInfo.seperation;
		}
		
		collisionInfoVector.push_back(collisionInfo);
	}
	else if (isCollideToHemisphere(capsuleB, -epaInfo.normal))
	{
		CollisionInfo collisionInfo;

		collisionInfo.normal = epaInfo.normal;
		collisionInfo.seperation = epaInfo.distance;

		if (glm::dot(capsuleB.axes[0], collisionInfo.normal) < 0)
		{
			glm::vec3 hemisphereCenter = capsuleB.center + capsuleB.axes[0] * 0.5f * capsuleB.height;
			collisionInfo.pointB = hemisphereCenter - collisionInfo.normal * capsuleB.radius;
			collisionInfo.pointA = collisionInfo.pointB + collisionInfo.normal * collisionInfo.seperation;
		}
		else
		{
			glm::vec3 hemisphereCenter = capsuleB.center - capsuleB.axes[0] * 0.5f * capsuleB.height;
			collisionInfo.pointB = hemisphereCenter - collisionInfo.normal * capsuleB.radius;
			collisionInfo.pointA = collisionInfo.pointB + collisionInfo.normal * collisionInfo.seperation;
		}
		
		collisionInfoVector.push_back(collisionInfo);
	}
	else
	{
		Face refFace = getCapsuleFace(capsuleA, epaInfo.normal);
		Face incFace = getCapsuleFace(capsuleB, -epaInfo.normal);

		std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);
		buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
	}
}

} // namespace ale