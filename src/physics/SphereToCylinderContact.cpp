#include "physics/SphereToCylinderContact.h"

namespace ale
{

SphereToCylinderContact::SphereToCylinderContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToCylinderContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(SphereToCylinderContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<SphereToCylinderContact *>(memory))SphereToCylinderContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 SphereToCylinderContact::supportA(const ConvexInfo &sphere, glm::vec3 dir)
{
	return sphere.center + dir * sphere.radius;
}

glm::vec3 SphereToCylinderContact::supportB(const ConvexInfo &cylinder, glm::vec3 dir)
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

void SphereToCylinderContact::findCollisionPoints(const ConvexInfo &sphere, const ConvexInfo &cylinder,
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