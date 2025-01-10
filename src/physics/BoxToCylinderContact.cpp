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
	float radius = cylinder.radius;

	// 1. 축 방향으로 윗면/아랫면 선택
	glm::vec3 base = center;

	if (glm::dot(dir, axis) > 0.0f)
	{
		base += 0.5f * height * axis;
	}
	else
	{
		base += -0.5f * height * axis;
	}

	// 2. 원형 단면의 가장 먼 점 계산
	glm::vec3 dirCircle = dir - glm::dot(dir, axis) * axis; // 축에 수직한 방향
	if (glm::length(dirCircle) > 0.0001f)
	{
		dirCircle = glm::normalize(dirCircle); // 정규화
	}
	else
	{
		dirCircle = glm::vec3(0.0f, 0.0f, 0.0f); // 축 방향으로만 정렬된 경우
	}
	glm::vec3 circlePoint = radius * dirCircle;

	return base + circlePoint;
}

void BoxToCylinderContact::findCollisionPoints(const ConvexInfo &box, const ConvexInfo &cylinder,
											   std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
											   std::vector<Simplex> &simplexVector)
{
	// std::cout << "box vs cylinder!!\n";

	// clipping
	Face refFace = getBoxFace(box, epaInfo.normal);
	Face incFace = getCylinderFace(cylinder, -epaInfo.normal);

	std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);

	// 폴리곤의 각 꼭지점 -> 충돌점 여러 개
	buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
}

} // namespace ale