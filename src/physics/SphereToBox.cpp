#include "physics/SphereToBoxContact.h"

namespace ale
{

SphereToBoxContact::SphereToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToBoxContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToBoxContact(fixtureA, fixtureB, indexA, indexB);
}

void SphereToBoxContact::evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB)
{
	/*
		64bit = 27bit(small proxyId) 	 |
				27bit(big proxyId) 		 |
				5bit(small contact part) |
				5bit(big contact part)
	*/
	static int64_t bitmask = 0xFFFFFFFF & ~0b11111;

	ManifoldPoint manifoldPoint;

	Shape *shapeA = m_fixtureA->getShape();
	Shape *shapeB = m_fixtureB->getShape();

	int64_t proxyIdA = m_fixtureA->getFixtureProxy()->proxyId;
	int64_t proxyIdB = m_fixtureB->getFixtureProxy()->proxyId;

	glm::vec3 worldCenterA = transformA.toMatrix() * glm::vec4(shapeA->localCenter, 1.0f);
	glm::vec3 localCenterB = shapeB->localCenter;
	glm::vec3 halfSizeB = shapeB->getLocalHalfSize();
	glm::mat4 matrix = transformB.toMatrix();
	glm::vec3 pointsB[8] = {matrix * glm::vec4(localCenterB - halfSizeB, 1.0f),
							matrix * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, -halfSizeB.y, -halfSizeB.z), 1.0f),
							matrix * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, halfSizeB.y, -halfSizeB.z), 1.0f),
							matrix * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, -halfSizeB.y, halfSizeB.z), 1.0f),
							matrix * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, halfSizeB.y, -halfSizeB.z), 1.0f),
							matrix * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, -halfSizeB.y, halfSizeB.z), 1.0f),
							matrix * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, halfSizeB.y, halfSizeB.z), 1.0f),
							matrix * glm::vec4(localCenterB + halfSizeB, 1.0f)};

	SphereToBoxInfo info;
	info.distance = std::numeric_limits<float>::max();
	;
	info.type = -1;

	for (int32_t i = 0; i < 8; i++)
	{
		glm::vec3 &point = pointsB[i];
		float distance = (point.x - worldCenterA.x) * (point.x - worldCenterA.x) +
						 (point.y - worldCenterA.y) * (point.y - worldCenterA.y) +
						 (point.z - worldCenterA.z) * (point.z - worldCenterA.z);

		if (distance < info.distance)
		{
			info.type = i;
			info.distance = distance;
			info.point = point;
			info.normal = glm::normalize(point - worldCenterA);
		}
	}

	static const int32_t linePointIndex[12][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 4}, {1, 5}, {2, 4},
												  {2, 6}, {3, 5}, {3, 6}, {4, 7}, {5, 7}, {6, 7}};

	for (int32_t i = 0; i < 12; i++)
	{
		int32_t idx1 = linePointIndex[i][0];
		int32_t idx2 = linePointIndex[i][1];
		getPointToEdgeDistance(worldCenterA, pointsB[idx1], pointsB[idx2], i + 8, info);
	}

	static const int32_t facePointIndex[6][4] = {{0, 1, 2, 4}, {0, 1, 3, 5}, {0, 2, 3, 6},
												 {1, 4, 5, 7}, {2, 4, 6, 7}, {3, 5, 6, 7}};

	for (int32_t i = 0; i < 6; i++)
	{
		int32_t idx1 = facePointIndex[i][0];
		int32_t idx2 = facePointIndex[i][1];
		int32_t idx3 = facePointIndex[i][2];
		int32_t idx4 = facePointIndex[i][3];
		getPointToFaceDistance(worldCenterA, pointsB[idx1], pointsB[idx2], pointsB[idx3], pointsB[idx4], i + 20, info);
	}

	float radius = shapeA->getLocalRadius();

	if (info.distance <= radius * radius)
	{
		manifoldPoint.point = info.point;
		manifoldPoint.normal = info.normal;
		if (info.type < 8)
			manifoldPoint.type = EManifoldType::FACE_A_TO_POINT_B;
		else if (info.type > 19)
			manifoldPoint.type = EManifoldType::FACE_A_TO_FACE_B;
		else
			manifoldPoint.type = EManifoldType::FACE_A_TO_EDGE_B;

		int64_t proxyIdA = m_fixtureA->getFixtureProxy()->proxyId;
		int64_t proxyIdB = m_fixtureB->getFixtureProxy()->proxyId;

		if (proxyIdA > proxyIdB)
		{
			int64_t tmp = proxyIdA;
			proxyIdA = proxyIdB;
			proxyIdB = tmp;
		}

		proxyIdA = (proxyIdA << 5) & bitmask;
		proxyIdB = (proxyIdB << 5) & bitmask;
		manifoldPoint.id = (proxyIdA << 32) | (proxyIdB << 10) | info.type;

		manifold.points.push_back(manifoldPoint);
	}
}

void SphereToBoxContact::getPointToEdgeDistance(const glm::vec3 &center, const glm::vec3 &p1, const glm::vec3 &p2,
												int32_t type, SphereToBoxInfo &info)
{
	glm::vec3 direction = p2 - p1;		  // 직선의 방향 벡터
	glm::vec3 pointToStart = center - p1; // 점과 직선 시작점 간의 벡터

	float length = glm::dot(direction, direction);

	float dotlength = glm::dot(pointToStart, direction);

	if (dotlength > length)
	{
		return;
	}

	glm::vec3 closestPoint = p1 + dotlength / length * direction;

	float distance = (closestPoint.x - center.x) * (closestPoint.x - center.x) +
					 (closestPoint.y - center.y) * (closestPoint.y - center.y) +
					 (closestPoint.z - center.z) * (closestPoint.z - center.z);

	if (distance < info.distance)
	{
		info.type = type;
		info.point = closestPoint;
		info.normal = glm::normalize(closestPoint - center);
		info.distance = distance;
	}
}

void SphereToBoxContact::getPointToFaceDistance(const glm::vec3 &center, const glm::vec3 &p1, const glm::vec3 &p2,
												const glm::vec3 &p3, const glm::vec3 &p4, int32_t type,
												SphereToBoxInfo &info)
{
	glm::vec3 v1 = p2 - p1;
	glm::vec3 v2 = p3 - p1;
	glm::vec3 normal = glm::normalize(glm::cross(v1, v2));

	// ax + by + cz + d = 0 평면 방정식의 d
	float d = -glm::dot(normal, p1);

	// 평면과 점 사이의 거리 공식
	float numerator = glm::dot(normal, center) + d;
	float distance = std::abs(numerator);
	distance = distance * distance;

	glm::vec3 closestPoint = center - numerator * normal;

	if (distance < info.distance)
	{
		info.type = type;
		info.point = closestPoint;
		info.normal = glm::normalize(closestPoint - center);
		info.distance = distance;
	}
}

} // namespace ale