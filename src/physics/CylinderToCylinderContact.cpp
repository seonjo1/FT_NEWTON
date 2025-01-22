#include "physics/CylinderToCylinderContact.h"

namespace ale
{

CylinderToCylinderContact::CylinderToCylinderContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA,
													 int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *CylinderToCylinderContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(CylinderToCylinderContact));
	if (memory == nullptr)
	{
		throw std::runtime_error("failed to allocate block");
	}
	return new (static_cast<CylinderToCylinderContact *>(memory))
		CylinderToCylinderContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 CylinderToCylinderContact::supportA(const ConvexInfo &cylinder, glm::vec3 dir)
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

glm::vec3 CylinderToCylinderContact::supportB(const ConvexInfo &cylinder, glm::vec3 dir)
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

void CylinderToCylinderContact::findCollisionPoints(const ConvexInfo &cylinderA, const ConvexInfo &cylinderB,
													CollisionInfo &collisionInfo, EpaInfo &epaInfo,
													SimplexArray &simplexArray)
{
	// std::cout << "cylinder vs cylinder!!\n";
	// clipping
	Face refFace, incFace;

	setCylinderFace(refFace, cylinderA, epaInfo.normal);
	setCylinderFace(incFace, cylinderB, -epaInfo.normal);

	// for (int i = 0; i < refFace.vertices.size(); i++)
	// {
	// 	std::cout << "refFace[" << i << "]: (" << refFace.vertices[i].x << ", " << refFace.vertices[i].y << ", " <<
	// refFace.vertices[i].z << ")\n";
	// }

	// for (int i = 0; i < incFace.vertices.size(); i++)
	// {
	// 	std::cout << "incFace[" << i << "]: (" << incFace.vertices[i].x << ", " << incFace.vertices[i].y << ", " <<
	// incFace.vertices[i].z << ")\n";
	// }

	ContactPolygon contactPolygon;
	computeContactPolygon(contactPolygon, refFace, incFace);

	// 폴리곤의 각 꼭지점 -> 충돌점 여러 개
	buildManifoldFromPolygon(collisionInfo, refFace, incFace, contactPolygon, epaInfo);
}

} // namespace ale