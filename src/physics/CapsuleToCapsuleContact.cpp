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

glm::vec3 CapsuleToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
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
			collisionInfo.pointA = capsuleA.points[0] + epaInfo.normal * capsuleA.radius;
			collisionInfo.pointB = collisionInfo.pointA - collisionInfo.normal * collisionInfo.seperation;
		}
		else
		{
			collisionInfo.pointA = capsuleA.points[1] + epaInfo.normal * capsuleA.radius;
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
			collisionInfo.pointB = capsuleB.points[0] + collisionInfo.normal * capsuleB.radius;
			collisionInfo.pointA = collisionInfo.pointB - collisionInfo.normal * collisionInfo.seperation;
		}
		else
		{
			collisionInfo.pointB = capsuleB.points[1] + collisionInfo.normal * capsuleB.radius;
			collisionInfo.pointA = collisionInfo.pointB - collisionInfo.normal * collisionInfo.seperation;
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