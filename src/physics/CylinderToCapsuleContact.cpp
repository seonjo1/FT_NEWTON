#include "physics/CylinderToCapsuleContact.h"

namespace ale
{

CylinderToCapsuleContact::CylinderToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *CylinderToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(CylinderToCapsuleContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<CylinderToCapsuleContact *>(memory))
		CylinderToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
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

void CylinderToCapsuleContact::findCollisionPoints(const ConvexInfo &cylinder, const ConvexInfo &capsule,
												   CollisionInfo &collisionInfo, EpaInfo &epaInfo,
												   SimplexArray &simplexArray)
{
	if (isCollideToHemisphere(capsule, -epaInfo.normal))
	{
		collisionInfo.normal[0] = epaInfo.normal;
		collisionInfo.seperation[0] = epaInfo.distance;

		if (glm::dot(capsule.axes[0], collisionInfo.normal[0]) < 0)
		{
			glm::vec3 hemisphereCenter = capsule.center + capsule.axes[0] * 0.5f * capsule.height;
			collisionInfo.pointB[0] = hemisphereCenter - collisionInfo.normal[0] * capsule.radius;
			collisionInfo.pointA[0] = collisionInfo.pointB[0] + collisionInfo.normal[0] * collisionInfo.seperation[0];
		}
		else
		{
			glm::vec3 hemisphereCenter = capsule.center - capsule.axes[0] * 0.5f * capsule.height;
			collisionInfo.pointB[0] = hemisphereCenter - collisionInfo.normal[0] * capsule.radius;
			collisionInfo.pointA[0] = collisionInfo.pointB[0] + collisionInfo.normal[0] * collisionInfo.seperation[0];
		}
		++collisionInfo.size;
	}
	else
	{
		Face refFace = getCylinderFace(cylinder, epaInfo.normal);
		Face incFace = getCapsuleFace(capsule, -epaInfo.normal);

		std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);
		buildManifoldFromPolygon(collisionInfo, refFace, incFace, contactPolygon, epaInfo);
	}
}

} // namespace ale