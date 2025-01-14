#include "physics/CylinderToCapsuleContact.h"

namespace ale
{

CylinderToCapsuleContact::CylinderToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *CylinderToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new CylinderToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 CylinderToCapsuleContact::supportA(const ConvexInfo &cylinder, glm::vec3 dir)
{
	// 원기둥 정보
	glm::vec3 center = cylinder.center;				   // 중심점
	glm::vec3 axis = glm::normalize(cylinder.axes[0]); // 높이 축 (단위 벡터)
	float height = cylinder.height;

	// 1. 축 방향으로 윗면/아랫면 선택

	bool isUpSide = true;
	float dotResult = glm::dot(dir, axis);

	if (dotResult < 0)
	{
		isUpSide = false;
	}

	glm::vec3 circleDir = dir - glm::dot(dir, axis) * axis; // 축에 수직한 방향

	if (glm::length2(circleDir) > 1e-8f)
	{
		circleDir = glm::normalize(circleDir); // 정규화
		
		int32_t maxIdx;
		int32_t segments = 20;

		float max = -FLT_MAX;
		for (int32_t i = 0; i < segments; ++i)
		{
			dotResult = glm::dot(cylinder.points[i], dir);
			if (dotResult > max)
			{
				maxIdx = i;
				max = dotResult;
			}
		}

		if (isUpSide == false)
		{
			maxIdx += segments;
		}

		return cylinder.points[maxIdx];
	}
	else
	{
		if (isUpSide)
		{
			return center + cylinder.axes[0] * height * 0.5f;
		}
		else
		{
			return center - cylinder.axes[0] * height * 0.5f;
		}
	}
}

glm::vec3 CylinderToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
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

void CylinderToCapsuleContact::findCollisionPoints(const ConvexInfo &cylinder, const ConvexInfo &capsule,
												   std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
												   std::vector<Simplex> &simplexVector)
{
	if (isCollideToHemisphere(capsule, -epaInfo.normal))
	{
		CollisionInfo collisionInfo;

		collisionInfo.normal = epaInfo.normal;
		collisionInfo.seperation = epaInfo.distance;
		if (glm::dot(capsule.axes[0], collisionInfo.normal) < 0)
		{
			collisionInfo.pointB = capsule.points[0] + collisionInfo.normal * capsule.radius;
			collisionInfo.pointA = collisionInfo.pointB - collisionInfo.normal * collisionInfo.seperation;
		}
		else
		{
			collisionInfo.pointB = capsule.points[1] + collisionInfo.normal * capsule.radius;
			collisionInfo.pointA = collisionInfo.pointB - collisionInfo.normal * collisionInfo.seperation;
		}

		collisionInfoVector.push_back(collisionInfo);
	}
	else
	{
		Face refFace = getCylinderFace(cylinder, epaInfo.normal);
		Face incFace = getCapsuleFace(capsule, -epaInfo.normal);

		std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);
		buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
	}
}

} // namespace ale