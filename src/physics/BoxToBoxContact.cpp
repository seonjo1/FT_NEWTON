#include "physics/BoxToBoxContact.h"

namespace ale
{

BoxToBoxContact::BoxToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *BoxToBoxContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new BoxToBoxContact(fixtureA, fixtureB, indexA, indexB);
}

void BoxToBoxContact::evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB)
{
	/*
		64bit = 27bit(small proxyId) 	 |
				27bit(big proxyId) 		 |
				5bit(small contact part) |
				5bit(big contact part)
	*/
	static int64_t bitmask = 0xFFFFFFFF & ~0b11111;

	Shape *shapeA = m_fixtureA->getShape();
	Shape *shapeB = m_fixtureB->getShape();

	// 박스 A 정의
	glm::vec3 localCenterA = shapeA->localCenter;
	glm::vec3 halfSizeA = shapeA->getLocalHalfSize();
	glm::mat4 matrixA = transformA.toMatrix();
	glm::vec3 worldCenterA = matrixA * glm::vec4(localCenterA, 1.0f);
	std::vector<glm::vec3> pointsA = {
		matrixA * glm::vec4(localCenterA + glm::vec3(-halfSizeA.x, -halfSizeA.y, halfSizeA.z), 1.0f),
		matrixA * glm::vec4(localCenterA + glm::vec3(halfSizeA.x, -halfSizeA.y, halfSizeA.z), 1.0f),
		matrixA * glm::vec4(localCenterA + glm::vec3(-halfSizeA.x, halfSizeA.y, halfSizeA.z), 1.0f),
		matrixA * glm::vec4(localCenterA - halfSizeA, 1.0f),
		matrixA * glm::vec4(localCenterA + halfSizeA, 1.0f),
		matrixA * glm::vec4(localCenterA + glm::vec3(halfSizeA.x, -halfSizeA.y, -halfSizeA.z), 1.0f),
		matrixA * glm::vec4(localCenterA + glm::vec3(-halfSizeA.x, halfSizeA.y, -halfSizeA.z), 1.0f),
		matrixA * glm::vec4(localCenterA + glm::vec3(halfSizeA.x, halfSizeA.y, -halfSizeA.z), 1.0f)};

	std::vector<glm::vec3> axesA = {
		glm::normalize(pointsA[1] - pointsA[0]),
		glm::normalize(pointsA[2] - pointsA[0]),
		glm::normalize(pointsA[3] - pointsA[0]),
	};

	// 박스 B 정의
	glm::vec3 localCenterB = shapeB->localCenter;
	glm::vec3 halfSizeB = shapeB->getLocalHalfSize();
	glm::mat4 matrixB = transformB.toMatrix();
	std::vector<glm::vec3> pointsB = {
		matrixB * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, -halfSizeB.y, halfSizeB.z), 1.0f),
		matrixB * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, -halfSizeB.y, halfSizeB.z), 1.0f),
		matrixB * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, halfSizeB.y, halfSizeB.z), 1.0f),
		matrixB * glm::vec4(localCenterB - halfSizeB, 1.0f),
		matrixB * glm::vec4(localCenterB + halfSizeB, 1.0f),
		matrixB * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, -halfSizeB.y, -halfSizeB.z), 1.0f),
		matrixB * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, halfSizeB.y, -halfSizeB.z), 1.0f),
		matrixB * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, halfSizeB.y, -halfSizeB.z), 1.0f)};

	std::vector<glm::vec3> axesB = {
		glm::normalize(pointsB[1] - pointsB[0]),
		glm::normalize(pointsB[2] - pointsB[0]),
		glm::normalize(pointsB[3] - pointsB[0]),
	};

	// for (int i = 0; i < 8; i++)
	// {
	// 	std::cout << "pointsA[" << i << "]: " << pointsA[i].x << " " << pointsA[i].y << " " << pointsA[i].z << "\n";
	// }

	// for (int i = 0; i < 8; i++)
	// {
	// 	std::cout << "pointsB[" << i << "]: " << pointsB[i].x << " " << pointsB[i].y << " " << pointsB[i].z << "\n";
	// }

	// 검사할 축 생성
	std::vector<glm::vec3> axes = {axesA[0], axesA[1], axesA[2], axesB[0], axesB[1], axesB[2]};
	// int i = 0;
	for (const glm::vec3 &axisA : axesA)
	{
		for (const glm::vec3 &axisB : axesB)
		{
			glm::vec3 crossAxis = glm::cross(axisA, axisB);
			if (glm::length(crossAxis) < 1e-6f)
			{
				crossAxis = glm::vec3(0.0f);
			}
			else
			{
				crossAxis = glm::normalize(crossAxis);
			}
			axes.push_back(crossAxis);
			// std::cout << "axes[" << i << "]: " << axes[i].x << " " << axes[i].y << " " << axes[i].z << "\n";
			// i++;
		}
	}

	// 충돌 판정
	BoxToBoxInfo info = boxToBoxSAT(pointsA, pointsB, axes);

	// 충돌 o
	if (info.collision)
	{
		ManifoldPoint manifoldPoint;
		if (info.axisType < 3)
		{
			fillFaceToPointInfo(info, pointsA, pointsB);
			manifoldPoint.type = EManifoldType::FACE_A_TO_POINT_B;
		}
		else if (info.axisType > 5)
		{
			fillEdgeToEdgeInfo(info, pointsA, pointsB);
			manifoldPoint.type = EManifoldType::EDGE_A_TO_EDGE_B;
		}
		else
		{
			fillPointToFaceInfo(info, pointsA, pointsB);
			manifoldPoint.type = EManifoldType::POINT_A_TO_FACE_B;
		}

		manifoldPoint.pointA = info.pointA;
		manifoldPoint.pointB = info.pointA - info.normal * info.overlap;
		manifoldPoint.normal = info.normal;
		manifoldPoint.seperation = info.overlap;

		// std::cout << "BoxToBox\n";
		// std::cout << "pointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " " << manifoldPoint.pointA.z << "\n";
		// std::cout << "pointB: " << manifoldPoint.pointB.x << " " << manifoldPoint.pointB.y << " " << manifoldPoint.pointB.z << "\n";
		// std::cout << "normal: " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " " << manifoldPoint.normal.z << "\n";
		// std::cout << "overlap: " << manifoldPoint.seperation << "\n";
		// std::cout << "type: " << info.axisType << "\n";

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
		manifoldPoint.id = (proxyIdA << 32) | (proxyIdB << 10) | (info.typeA << 5) | (info.typeB);

		manifold.points.push_back(manifoldPoint);
	}
}

BoxToBoxInfo BoxToBoxContact::boxToBoxSAT(const std::vector<glm::vec3> &pointsA, const std::vector<glm::vec3> &pointsB,
										  const std::vector<glm::vec3> &axes)
{
	BoxToBoxInfo info;
	info.collision = true; // 초기값
	info.overlap = std::numeric_limits<float>::max();

	for (int32_t i = 0; i < 15; i++)
	{
		glm::vec3 axis = axes[i];
		if (glm::length(axis) == 0)
		{
			continue;
		}

		if (!isOverlapped(info, axis, pointsA, pointsB, i))
		{
			info.collision = false;
			return info; // 충돌 없음
		}
	}

	return info; // 충돌 정보 반환
}

bool BoxToBoxContact::isOverlapped(BoxToBoxInfo &info, const glm::vec3 &axis, const std::vector<glm::vec3> &pointsA,
								   const std::vector<glm::vec3> &pointsB, int32_t axisType)
{
	// std::cout << "overlap start\n";
	// 박스 A와 B의 최소/최대 투영 값 계산
	float minA = std::numeric_limits<float>::max();
	float maxA = std::numeric_limits<float>::lowest();
	int32_t minTypeA = -1;
	int32_t maxTypeA = -1;

	// axis에 boxA 점들 투영
	for (int i = 0; i < 8; i++)
	{
		const glm::vec3 &point = pointsA[i];
		float projection = glm::dot(point, axis);

		if (projection < minA)
		{
			minA = projection;
			minTypeA = i;
		}

		if (projection > maxA)
		{
			maxA = projection;
			maxTypeA = i;
		}
	}

	// 박스 A와 B의 최소/최대 투영 값 계산
	float minB = std::numeric_limits<float>::max();
	float maxB = std::numeric_limits<float>::lowest();
	int32_t minTypeB = -1;
	int32_t maxTypeB = -1;

	// axis에 boxB 점들 투영
	for (int i = 0; i < 8; i++)
	{
		const glm::vec3 &point = pointsB[i];
		float projection = glm::dot(point, axis);

		if (projection < minB)
		{
			minB = projection;
			minTypeB = i;
		}

		if (projection > maxB)
		{
			maxB = projection;
			maxTypeB = i;
		}
	}
	// std::cout << "minA: " << minA << "\n";
	// std::cout << "maxA: " << maxA << "\n";
	// std::cout << "minB: " << minB << "\n";
	// std::cout << "maxB: " << maxB<< "\n";
	// 충돌하지 않음
	if (maxA < minB || maxB < minA)
	{
		return false;
	}
	// 충돌 발생 - 겹침 길이 계산
	float overlap = 0.0f;
	if ((minA < minB && maxB < maxA) || (minB < minA && maxA < maxB))
	{
		if (maxB - minA < maxA - minB)
		{
			overlap = maxB - minA;
			if (overlap < info.overlap)
			{
				changeInfo(info, overlap, minTypeA, maxTypeB, axisType);
			}
		}
		else
		{
			overlap = maxA - minB;
			if (overlap < info.overlap)
			{
				changeInfo(info, overlap, maxTypeA, minTypeB, axisType);
			}
		}
	}
	else if (minA <= minB && maxA <= maxB)
	{
		overlap = maxA - minB;
		if (overlap < info.overlap)
		{
			changeInfo(info, overlap, maxTypeA, minTypeB, axisType);
		}
	}
	else if (minB <= minA && maxB <= maxA)
	{
		overlap = maxB - minA;
		if (overlap < info.overlap)
		{
			changeInfo(info, overlap, minTypeA, maxTypeB, axisType);
		}
	}
	// std::cout << "axisType[" << axisType << "] overlap: " << overlap << "\n";
	return true;
}

void BoxToBoxContact::changeInfo(BoxToBoxInfo &info, float overlap, int32_t typeA, int32_t typeB, int32_t axisType)
{
	info.overlap = overlap;
	info.typeA = typeA;
	info.typeB = typeB;
	info.axisType = axisType;
}

void BoxToBoxContact::fillFaceToPointInfo(BoxToBoxInfo &info, std::vector<glm::vec3> &pointsA,
										  std::vector<glm::vec3> &pointsB)
{
	static const std::vector<std::vector<std::vector<int32_t>>> faceVector = {
		{{0, 2, 3, 6}, {1, 5, 4, 7}}, {{0, 3, 1, 5}, {2, 4, 6, 7}}, {{0, 1, 2, 4}, {3, 6, 5, 7}}};
	std::vector<int32_t> faceA = isContainPoint(faceVector[info.axisType][0], info.typeA)
									 ? faceVector[info.axisType][0]
									 : faceVector[info.axisType][1];
	glm::vec3 &pointB = pointsB[info.typeB];
	info.normal =
		glm::normalize(glm::cross(pointsA[faceA[1]] - pointsA[faceA[0]], pointsA[faceA[2]] - pointsA[faceA[0]]));
	info.pointA = pointB - info.normal * info.overlap;
	// std::cout << "axisType 0 ~ 2\n"; 
}

void BoxToBoxContact::fillEdgeToEdgeInfo(BoxToBoxInfo &info, std::vector<glm::vec3> &pointsA,
										 std::vector<glm::vec3> &pointsB)
{
	static const std::vector<std::vector<std::vector<int32_t>>> edgeVector = {
		{{0, 1}, {2, 4}, {3, 5}, {6, 7}}, {{0, 2}, {1, 4}, {3, 6}, {5, 7}}, {{0, 3}, {1, 5}, {2, 6}, {4, 7}}};

	int32_t axisTypeA = (info.axisType - 6) / 3;
	int32_t axisTypeB = (info.axisType - 6) % 3;
	std::vector<glm::vec3> edgeA, projectedEdgeA;
	std::vector<glm::vec3> edgeB, projectedEdgeB;

	for (int i = 0; i < 4; i++)
	{
		if (isContainPoint(edgeVector[axisTypeA][i], info.typeA))
		{
			edgeA.push_back(pointsA[edgeVector[axisTypeA][i][0]]);
			edgeA.push_back(pointsA[edgeVector[axisTypeA][i][1]]);
			break;
		}
	}

	for (int i = 0; i < 4; i++)
	{
		if (isContainPoint(edgeVector[axisTypeB][i], info.typeB))
		{
			edgeB.push_back(pointsB[edgeVector[axisTypeB][i][0]]);
			edgeB.push_back(pointsB[edgeVector[axisTypeB][i][1]]);
			break;
		}
	}

	glm::vec3 normal = glm::normalize(glm::cross(edgeA[1] - edgeA[0], edgeB[1] - edgeB[0]));

	if (glm::dot(normal, edgeA[0] - edgeB[0]) < 0)
	{
		normal = -normal;
	}

	projectedEdgeA = getProjectedEdge(normal, edgeA);
	projectedEdgeB = getProjectedEdge(normal, edgeB);

	float ratio = findIntersectionRatio(projectedEdgeA, projectedEdgeB);

	info.normal = normal;
	info.pointA = edgeA[0] + (edgeA[1] - edgeA[0]) * ratio;
	// std::cout << "axis type: 6 ~ 15\n";
}

void BoxToBoxContact::fillPointToFaceInfo(BoxToBoxInfo &info, std::vector<glm::vec3> &pointsA,
										  std::vector<glm::vec3> &pointsB)
{
	static const std::vector<std::vector<std::vector<int32_t>>> faceVector = {
		{{0, 2, 3, 6}, {1, 5, 4, 7}}, {{0, 3, 1, 5}, {2, 4, 6, 7}}, {{0, 1, 2, 4}, {3, 6, 5, 7}}};

	int32_t axisType = info.axisType - 3;
	glm::vec3 &pointA = pointsA[info.typeA];
	std::vector<int32_t> faceB =
		isContainPoint(faceVector[axisType][0], info.typeB) ? faceVector[axisType][0] : faceVector[axisType][1];
	info.pointA = pointA;
	info.normal =
		-glm::normalize(glm::cross(pointsB[faceB[1]] - pointsB[faceB[0]], pointsB[faceB[2]] - pointsB[faceB[0]]));
	// std::cout << "axisType 3 ~ 5\n"; 
}

bool BoxToBoxContact::isContainPoint(std::vector<int32_t> points, int32_t point)
{
	for (int32_t p : points)
	{
		if (p == point)
		{
			return true;
		}
	}
	return false;
}

std::vector<glm::vec3> BoxToBoxContact::getProjectedEdge(const glm::vec3 &normal, const std::vector<glm::vec3> &edge)
{
	std::vector<glm::vec3> projectedEdge;

	for (int i = 0; i < 2; i++)
	{
		float px = edge[i].x - ((normal.x * edge[i].x + normal.y * edge[i].y + normal.z * edge[i].z) /
								(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z)) *
								   normal.x;
		float py = edge[i].y - ((normal.x * edge[i].x + normal.y * edge[i].y + normal.z * edge[i].z) /
								(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z)) *
								   normal.y;
		float pz = edge[i].z - ((normal.x * edge[i].x + normal.y * edge[i].y + normal.z * edge[i].z) /
								(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z)) *
								   normal.z;
		projectedEdge.emplace_back(px, py, pz);
	}

	return projectedEdge;
}

float BoxToBoxContact::findIntersectionRatio(const std::vector<glm::vec3> &edgeA, const std::vector<glm::vec3> &edgeB)
{
	/*
		A(t) = edgeA[0] + dA * t;
		B(u) = edgeB[0] + dB * u;
		r = edgeB[0] - edgeA[0];

		교점 : A(t) == B(u)
		A(t) = B(u) -> edgeA[0] + dA * t = edgeB[0] + dB * u;
		r = dA * t - dB * u;

		rx = dAx * t - dBx * u;  ... (1)
		ry = dAy * t - dBy * u;  ... (2)
		rz = dAz * t - dBz * u;  ... (3)

		=> (1) - (2) * (dBx / dBy)
		t = (rx - (ry * dBx) / dBy) / (dAx - (dAy * dBx) / dBy)

	*/
	glm::vec3 dA = edgeA[1] - edgeA[0]; // 선분 A의 방향 벡터
	glm::vec3 dB = edgeB[1] - edgeB[0]; // 선분 B의 방향 벡터
	glm::vec3 r = edgeB[0] - edgeA[0];

	float t;

	// (1), (2) 공식에서 분모가 0에 가까운 경우
	if (glm::abs(dB.y) < 1e-6f)
	{
		// (2), (3) 공식에서 분모가 0에 가까운 경우
		if (glm::abs(dB.z) < 1e-6f)
		{
			// (3), (1) 공식
			t = (r.z - (r.x * dB.z) / dB.x) / (dA.z - (dA.x * dB.z) / dB.x);
		}
		else
		{
			// (2), (3) 공식
			t = (r.y - (r.z * dB.y) / dB.z) / (dA.y - (dA.z * dB.y) / dB.z);
		}
	}
	else
	{
		// (1), (2) 공식
		t = (r.x - (r.y * dB.x) / dB.y) / (dA.x - (dA.y * dB.x) / dB.y);
	}

	return t;
}

} // namespace ale