#include "physics/BoxToCylinderContact.h"

namespace ale
{

BoxToCylinderContact::BoxToCylinderContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *BoxToCylinderContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new BoxToCylinderContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 BoxToCylinderContact::supportA(const ConvexInfo &box, glm::vec3 dir)
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

glm::vec3 BoxToCylinderContact::supportB(const ConvexInfo &cylinder, glm::vec3 dir)
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

void BoxToCylinderContact::findCollisionPoints(const ConvexInfo &box, const ConvexInfo &cylinder,
											   std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
											   std::vector<Simplex> &simplexVector)
{
	// std::cout << "box vs cylinder!!\n";

	// clipping
	Face refFace = getBoxFace(box, epaInfo.normal);
	Face incFace = getCylinderFace(cylinder, -epaInfo.normal);

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

	std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);

	// 폴리곤의 각 꼭지점 -> 충돌점 여러 개
	buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
}

} // namespace ale