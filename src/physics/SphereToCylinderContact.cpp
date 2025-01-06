#include "physics/SphereToCylinderContact.h"

namespace ale
{

SphereToCylinderContact::SphereToCylinderContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToCylinderContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToCylinderContact(fixtureA, fixtureB, indexA, indexB);
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
	float radius = cylinder.radius;

	// 1. 축 방향으로 윗면/아랫면 선택
	float isUpperDir = glm::dot(dir, axis);
	glm::vec3 base = center;

	if (isUpperDir)
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