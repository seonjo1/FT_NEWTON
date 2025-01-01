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

	BoxInfo boxA;
	BoxInfo boxB;

	// 박스 A 정의
	glm::vec3 localCenterA = shapeA->localCenter;
	glm::vec3 halfSizeA = shapeA->getLocalHalfSize();
	glm::mat4 matrixA = transformA.toMatrix();
	boxA.center = matrixA * glm::vec4(localCenterA, 1.0f);
	boxA.halfSize = halfSizeA;
	boxA.points = {matrixA * glm::vec4(localCenterA - halfSizeA, 1.0f),
				   matrixA * glm::vec4(localCenterA + glm::vec3(halfSizeA.x, -halfSizeA.y, -halfSizeA.z), 1.0f),
				   matrixA * glm::vec4(localCenterA + glm::vec3(-halfSizeA.x, halfSizeA.y, -halfSizeA.z), 1.0f),
				   matrixA * glm::vec4(localCenterA + glm::vec3(-halfSizeA.x, -halfSizeA.y, halfSizeA.z), 1.0f),
				   matrixA * glm::vec4(localCenterA + glm::vec3(halfSizeA.x, halfSizeA.y, -halfSizeA.z), 1.0f),
				   matrixA * glm::vec4(localCenterA + glm::vec3(halfSizeA.x, -halfSizeA.y, halfSizeA.z), 1.0f),
				   matrixA * glm::vec4(localCenterA + glm::vec3(-halfSizeA.x, halfSizeA.y, halfSizeA.z), 1.0f),
				   matrixA * glm::vec4(localCenterA + halfSizeA, 1.0f)};
	glm::vec3 axisX = glm::normalize(boxA.points[1] - boxA.points[0]);
	glm::vec3 axisY = glm::normalize(boxA.points[2] - boxA.points[0]);
	glm::vec3 axisZ = glm::normalize(boxA.points[3] - boxA.points[0]);

	boxA.axes = {axisX, axisY, axisZ};

	for (int k = 0; k < 8; k++)
	{
		std::cout << "pointA[" << k << "]: " << boxA.points[k].x << " " << boxA.points[k].y << " " << boxA.points[k].z
				  << "\n";
	}

	// 박스 B 정의
	glm::vec3 localCenterB = shapeB->localCenter;
	glm::vec3 halfSizeB = shapeB->getLocalHalfSize();
	glm::mat4 matrixB = transformB.toMatrix();
	boxB.center = matrixB * glm::vec4(localCenterB, 1.0f);
	boxB.halfSize = halfSizeB;
	boxB.points = {matrixB * glm::vec4(localCenterB - halfSizeB, 1.0f),
				   matrixB * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, -halfSizeB.y, -halfSizeB.z), 1.0f),
				   matrixB * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, halfSizeB.y, -halfSizeB.z), 1.0f),
				   matrixB * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, -halfSizeB.y, halfSizeB.z), 1.0f),
				   matrixB * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, halfSizeB.y, -halfSizeB.z), 1.0f),
				   matrixB * glm::vec4(localCenterB + glm::vec3(halfSizeB.x, -halfSizeB.y, halfSizeB.z), 1.0f),
				   matrixB * glm::vec4(localCenterB + glm::vec3(-halfSizeB.x, halfSizeB.y, halfSizeB.z), 1.0f),
				   matrixB * glm::vec4(localCenterB + halfSizeB, 1.0f)};

	axisX = glm::normalize(boxB.points[1] - boxB.points[0]);
	axisY = glm::normalize(boxB.points[2] - boxB.points[0]);
	axisZ = glm::normalize(boxB.points[3] - boxB.points[0]);

	boxB.axes = {axisX, axisY, axisZ};

	for (int k = 0; k < 8; k++)
	{
		std::cout << "pointB[" << k << "]: " << boxB.points[k].x << " " << boxB.points[k].y << " " << boxB.points[k].z
				  << "\n";
	}

	std::vector<Simplex> simplex;
	bool isCollide = getGjkResult(boxA, boxB, simplex);

	if (isCollide)
	{
		std::cout << "GJK COLLIDE OK\n";
		std::vector<CollisionPoints> collisionPointsVector = getEpaResult(boxA, boxB, simplex);
		for (const CollisionPoints &collisionPoints : collisionPointsVector)
		{

			ManifoldPoint manifoldPoint;
			manifoldPoint.pointA = collisionPoints.pointA;
			manifoldPoint.pointB = collisionPoints.pointB;
			manifoldPoint.normal = collisionPoints.normal;
			manifoldPoint.seperation = collisionPoints.seperation;
			// std::cout << "\n\nResult!!\npointA: " << manifoldPoint.pointA.x << " " << manifoldPoint.pointA.y << " "
			// 		<< manifoldPoint.pointA.z << "\n";
			// std::cout << "pointB: " << manifoldPoint.pointB.x << " " << manifoldPoint.pointB.y << " "
			// 		<< manifoldPoint.pointB.z << "\n";
			// std::cout << "normal: " << manifoldPoint.normal.x << " " << manifoldPoint.normal.y << " "
			// 		<< manifoldPoint.normal.z << "\n";
			// std::cout << "seperation: " << manifoldPoint.seperation << "\n\n";

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
			manifoldPoint.id = (proxyIdA << 32) | (proxyIdB << 10);

			manifold.points.push_back(manifoldPoint);
		}
	}
}

glm::vec3 BoxToBoxContact::supportBox(const BoxInfo &box, glm::vec3 dir)
{
	float dotAxes[3] = {glm::dot(box.axes[0], dir) > 0 ? 1.0f : -1.0f, glm::dot(box.axes[1], dir) > 0 ? 1.0f : -1.0f,
						glm::dot(box.axes[2], dir) > 0 ? 1.0f : -1.0f};

	glm::vec3 point = box.center;
	for (int i = 0; i < 3; ++i)
	{
		point += box.axes[i] * (dotAxes[i] * box.halfSize[i]);
	}

	// std::cout << "support dir: " << dir.x << " " << dir.y << " " << dir.z << "\n";
	// std::cout << "support box point: " << point.x  << " " << point.y << " " << point.z << "\n";
	return point;
}

std::vector<glm::vec4> BoxToBoxContact::getCandidates(const BoxInfo &box, const glm::vec3 &dir)
{
	std::vector<glm::vec4> candidates;

	for (const glm::vec3 &point : box.points)
	{
		float distance = glm::dot(point, dir);

		candidates.push_back(glm::vec4(point, distance));
	}

	std::sort(candidates.begin(), candidates.end(),
			  [](const glm::vec4 &v1, const glm::vec4 &v2) { return v1.w > v2.w; });

	// for (int i = 0; i < candidates.size(); i++)
	// {
	// 	std::cout << "candidates[" << i << "]: " << candidates[i].x << " " << candidates[i].y << " " << candidates[i].z
	// << "\n"; 	std::cout << "distance: " << candidates[i].w << "\n";
	// }

	return candidates;
}

Simplex BoxToBoxContact::getSupportPoint(const BoxInfo &boxA, const BoxInfo &boxB, glm::vec3 &dir)
{
	Simplex simplex;
	simplex.a = supportBox(boxA, dir);
	simplex.b = supportBox(boxB, -dir);
	simplex.diff = simplex.a - simplex.b;
	return simplex;
}

bool BoxToBoxContact::handleLineSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir)
{
	std::cout << "Line GJK\n";
	glm::vec3 &a = simplexVector[0].diff;
	glm::vec3 &b = simplexVector[1].diff;

	glm::vec3 ab = b - a;
	glm::vec3 ao = -a;

	if (isSameDirection(ab, ao))
	{
		std::cout << "sameDirection!!\n";
		glm::vec3 tmpAxis(0, 1, 0);
		if (isSameDirection(ab, tmpAxis))
		{
			tmpAxis = glm::vec3(1, 0, 0);
			dir = glm::normalize(glm::cross(ab, tmpAxis));
		}
		else
		{
			dir = glm::normalize(glm::cross(ab, tmpAxis));
		}
	}
	else
	{
		dir = glm::normalize(glm::cross(glm::cross(ab, ao), ab));
	}

	return false;
}

bool BoxToBoxContact::handleTriangleSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir)
{
	std::cout << "Triangle GJK\n";
	glm::vec3 &a = simplexVector[0].diff;
	glm::vec3 &b = simplexVector[1].diff;
	glm::vec3 &c = simplexVector[2].diff;

	glm::vec3 ab = b - a;
	glm::vec3 ac = c - a;
	glm::vec3 bc = c - b;
	glm::vec3 abc = glm::cross(ac, ab);

	// 면적이 없으면 반대 dir로 세 번째 점 다시 찾기기
	if (glm::length2(abc) == 0)
	{
		simplexVector.pop_back();
		dir = -dir;
		return false;
	}

	dir = glm::normalize(glm::cross(abc, bc));
	if (glm::dot(dir, -c) > 0.0f)
	{
		// a 제거
		simplexVector.erase(simplexVector.begin());
		return false;
	}

	dir = glm::normalize(glm::cross(ac, abc));
	if (glm::dot(dir, -c) > 0.0f)
	{
		// b 제거
		simplexVector.erase(simplexVector.begin() + 1);
		return false;
	}

	if (glm::dot(-a, abc) > 0.0f)
	{
		dir = glm::normalize(abc);
	}
	else
	{
		dir = glm::normalize(-abc);
	}

	return false;
}

bool BoxToBoxContact::handleTetrahedronSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir)
{
	std::cout << "Tetrahedron GJK\n";
	glm::vec3 &a = simplexVector[0].diff;
	glm::vec3 &b = simplexVector[1].diff;
	glm::vec3 &c = simplexVector[2].diff;
	glm::vec3 &d = simplexVector[3].diff;

	glm::vec3 abc = glm::cross((b - a), (c - a));
	// 부피가 없으면 반대 방향으로 4번째 점 다시 찾기기
	if (glm::dot((d - a), abc) == 0.0f)
	{
		simplexVector.pop_back();
		dir = -dir;
		return false;
	}

	// bcd 법선 방향에 원점 있는지 확인
	glm::vec3 bc = c - b;
	glm::vec3 bd = d - b;
	glm::vec3 bcd = glm::cross(bd, bc);
	if (glm::dot(bcd, a - c) > 0)
	{
		bcd = -bcd;
	}

	if (glm::dot(bcd, -d) > 0.0f)
	{
		simplexVector.erase(simplexVector.begin());
		dir = glm::normalize(bcd);
		return false;
	}

	// acd 법선 방향에 원점 있는지 확인
	glm::vec3 ca = a - c;
	glm::vec3 cd = d - c;
	glm::vec3 acd = glm::cross(cd, ca);
	if (glm::dot(acd, b - c) > 0)
	{
		acd = -acd;
	}

	if (glm::dot(acd, -d) > 0.0f)
	{
		simplexVector.erase(simplexVector.begin() + 1);
		dir = glm::normalize(acd);
		return false;
	}

	// abd 법선 방향에 원점 있는지 확인
	glm::vec3 ab = b - a;
	glm::vec3 ad = d - a;
	glm::vec3 abd = glm::cross(ad, ab);

	if (glm::dot(abd, c - a) > 0)
	{
		abd = -abd;
	}

	if (glm::dot(abd, -d) > 0.0f)
	{
		simplexVector.erase(simplexVector.begin() + 2);
		dir = glm::normalize(abd);
		return false;
	}

	return true;
}

// 가장 최근에 추가된 점 A = simplex.points.back() 로 가정
// simplex가 2, 3, 4개 점일 때 각각 처리 달라짐
bool BoxToBoxContact::handleSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir)
{
	switch (simplexVector.size())
	{
	case 2:
		return handleLineSimplex(simplexVector, dir);
	case 3:
		return handleTriangleSimplex(simplexVector, dir);
	case 4:
		return handleTetrahedronSimplex(simplexVector, dir);
	}
	return false;
}

bool BoxToBoxContact::isDuplicatedPoint(const std::vector<Simplex> &simplexVector, const glm::vec3 &supportPoint)
{
	int32_t size = simplexVector.size();
	for (int32_t i = 0; i < size; i++)
	{
		if (glm::length2(simplexVector[i].diff - supportPoint) < 1e-6f)
		{
			return true;
		}
	}
	return false;
}

bool BoxToBoxContact::getGjkResult(const BoxInfo &boxA, const BoxInfo &boxB, std::vector<Simplex> &simplexVector)
{
	// 첫 번째 support point 구하기
	glm::vec3 dir = glm::normalize(boxB.center - boxA.center);
	if (glm::length2(dir) < 1e-8f)
	{
		dir = glm::vec3(1.0f, 0.0f, 0.0f);
	}

	simplexVector.push_back(getSupportPoint(boxA, boxB, dir));
	glm::vec3 supportPoint = simplexVector.back().diff;

	// 두 번째 support point 구하기
	dir = glm::normalize(-supportPoint);

	while (true)
	{
		// 새로운 서포트 점
		Simplex simplex = getSupportPoint(boxA, boxB, dir);
		supportPoint = simplex.diff;

		// 만약 newSupport가 direction과 내적(dot)했을 때 0 이하라면
		// 더 이상 원점을 "방향 dir" 쪽에서 감쌀 수 없음 => 충돌X
		if (glm::dot(supportPoint, dir) < 0 || isDuplicatedPoint(simplexVector, supportPoint))
		{
			return false; // 교차하지 않음
		}

		// 심플렉스에 추가
		simplexVector.push_back(simplex);

		// 원점을 포함하는지 체크 및 simplex 갱신
		if (handleSimplex(simplexVector, dir))
		{
			// 원점 포함 => 충돌
			return true;
		}
	}

	return false;
}

bool BoxToBoxContact::isSimilarDirection(glm::vec3 v1, glm::vec3 v2)
{
	// std::cout << "v1: " << v1.x << " " << v1.y << " " << v1.z << "\n";
	// std::cout << "v2: " << v2.x << " " << v2.y << " " << v2.z << "\n";
	// std::cout << "dot : " << glm::dot(v1, v2) << "\n";
	return glm::dot(v1, v2) > 0.0f;
}

bool BoxToBoxContact::isSameDirection(glm::vec3 v1, glm::vec3 v2)
{
	return glm::length2(glm::cross(v1, v2)) < 1e-6f;
}

std::vector<CollisionPoints> BoxToBoxContact::getEpaResult(const BoxInfo &boxA, const BoxInfo &boxB,
														   std::vector<Simplex> &simplexVector)
{
	std::vector<int32_t> faces = {0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2};

	// GJK에서 구한 simplex들중 원점에서 가장 가까운 삼각형의 법선과 최소 거리
	std::vector<glm::vec4> normals;
	int32_t minFace = getFaceNormals(normals, simplexVector, faces);
	glm::vec3 minNormal;
	// std::cout << "minFace: " << minFace << "\n";
	float minDistance = FLT_MAX;

	while (minDistance == FLT_MAX)
	{
		// 최소 거리의 법선, 거리 쿼리
		minNormal = glm::vec3(normals[minFace]);
		minDistance = normals[minFace].w;

		// 최소 거리의 법선에 해당하는 supportPoint 쿼리
		Simplex simplex = getSupportPoint(boxA, boxB, minNormal);
		glm::vec3 supportPoint = simplex.diff;

		// 원점에서 supportPoint까지의 거리
		float supportDistance = glm::dot(minNormal, supportPoint);

		// supportPoint가 현재 minDistance보다 원점에서 더 멀리있는 경우
		// 다시 원점에서부터 최소거리의 삼각형을 찾음
		if (std::abs(supportDistance - minDistance) > 1e-8f && !isDuplicatedPoint(simplexVector, supportPoint))
		{
			minDistance = FLT_MAX;
			std::vector<std::pair<int32_t, int32_t>> uniqueEdges;
			// std::cout << "new supportPoint: " << supportPoint.x << " " << supportPoint.y << " " << supportPoint.z <<
			// "\n"; std::cout << "start make unique edges\n";
			for (int32_t i = 0; i < normals.size(); i++)
			{
				// std::cout << "normals[" << i << "]: " << normals[i].x << " " << normals[i].y << " " << normals[i].z
				// << "\n"; std::cout << "check same direction\n"; 새로운 점에서 해당 평면이 보이는지 판단. 면의
				// 중심에서 새로운 점까지의 벡터와 면의 법선벡터가 같은 방향인지 판단
				// std::cout << "this plane idx!!\n";
				// std::cout << faces[i] << " " << faces[i + 1] << " " << faces[i + 2] << "\n";
				// std::cout << "face 1: " << polytope[faces[i]].x << " " << polytope[faces[i]].y << " " <<
				// polytope[faces[i]].z << "\n"; std::cout << "face 2: " << polytope[faces[i + 1]].x << " " <<
				// polytope[faces[i + 1]].y << " " << polytope[faces[i + 1]].z << "\n"; std::cout << "face 3: " <<
				// polytope[faces[i + 2]].x << " " << polytope[faces[i + 2]].y << " " << polytope[faces[i + 2]].z <<
				// "\n";
				glm::vec3 center =
					(simplexVector[faces[i * 3]].diff + simplexVector[faces[i * 3 + 1]].diff + simplexVector[faces[i * 3 + 2]].diff) / 3.0f;
				if (isSimilarDirection(normals[i], supportPoint - center))
				{
					// std::cout << "add uniqueEdge\n";
					// std::cout << "normals in!!!\n";
					int32_t faceIdx = i * 3;

					// 해당 법선의 기존 삼각형의 edge들을 uniqueEdges에 저장
					// 만약 같은 edge가 2번 들어오면 사라질 edge로 판단하여 삭제
					// 1번만 들어오는 edge들만 모아서 새로운 점과 조합하여 새로운 삼각형 생성성
					addIfUniqueEdge(uniqueEdges, faces, faceIdx, faceIdx + 1);
					addIfUniqueEdge(uniqueEdges, faces, faceIdx + 1, faceIdx + 2);
					addIfUniqueEdge(uniqueEdges, faces, faceIdx + 2, faceIdx);

					faces[faceIdx + 2] = faces.back();
					faces.pop_back();
					faces[faceIdx + 1] = faces.back();
					faces.pop_back();
					faces[faceIdx] = faces.back();
					faces.pop_back();

					normals[i] = normals.back(); // pop-erase
					normals.pop_back();

					i--;
				}
			}

			// std::cout << "make new faces\n";
			// uniqueEdge에 있는 edge들과 새로운 점을 조함하여 face 생성
			std::vector<int32_t> newFaces;
			// std::cout << "uniqueEdges size: " << uniqueEdges.size() << "\n";
			for (auto [edgeIndex1, edgeIndex2] : uniqueEdges)
			{
				newFaces.push_back(edgeIndex1);
				newFaces.push_back(edgeIndex2);
				newFaces.push_back(simplexVector.size());
			}
			// 새로 추가되는 면이 없다면 종료료
			if (newFaces.size() == 0)
			{
				minNormal = glm::vec3(normals[minFace]);
				minDistance = normals[minFace].w;
				continue;
			}
			// 새로운 점 추가
			simplexVector.push_back(simplex);

			// std::cout << "renew polytope\n";
			// for (const glm::vec3& poly : polytope)
			// {
			// 	std::cout << poly.x << " " << poly.y << " " << poly.z << "\n";
			// }

			// 새로운 삼각형의 normal 벡터들과 최소 거리 쿼리
			std::vector<glm::vec4> newNormals;
			int32_t newMinFace = getFaceNormals(newNormals, simplexVector, newFaces);
			float oldMinDistance = FLT_MAX;

			// 기존 삼각형들 중 가장 거리가 짧은 삼각형 쿼리
			for (int32_t i = 0; i < normals.size(); i++)
			{
				if (normals[i].w < oldMinDistance)
				{
					oldMinDistance = normals[i].w;
					minFace = i;
				}
			}

			// 새로운 삼각형들중 가장 짧은 거리가 존재하면 해당 face를 minFace로 설정
			if (newNormals[newMinFace].w < oldMinDistance)
			{
				minFace = newMinFace + normals.size();
			}

			// 새로운 face, normal 추가
			// std::cout << "\n\nBefore faces\n";
			// for (int i = 0; i < faces.size(); i = i + 3)
			// {
			// 	std::cout << "\ntriangle[" << i / 3 << "]\n";
			// 	std::cout << faces[i] << " " << faces[i + 1] << " " << faces[i + 2] << "\n";
			// }
			faces.insert(faces.end(), newFaces.begin(), newFaces.end());
			normals.insert(normals.end(), newNormals.begin(), newNormals.end());
			// std::cout << "\n\nAfter faces\n";
			// for (int i = 0; i < faces.size(); i = i + 3)
			// {
			// 	std::cout << "\ntriangle[" << i / 3 << "]\n";
			// 	std::cout << faces[i] << " " << faces[i + 1] << " " << faces[i + 2] << "\n";
			// }
		}
	}

	// 바리센트릭 방법
	std::vector<CollisionPoints> pointsVector;

	float u, v, w;
	int32_t idx0 = faces[minFace * 3];
	int32_t idx1 = faces[minFace * 3 + 1];
	int32_t idx2 = faces[minFace * 3 + 2];

	glm::vec3 projectedO = minNormal * minDistance;
	barycentric(simplexVector[idx0].diff, simplexVector[idx1].diff, simplexVector[idx2].diff, projectedO, u, v, w);
	
	glm::vec3 pointA = simplexVector[idx0].a * u + simplexVector[idx1].a * v + simplexVector[idx2].a * w;
	glm::vec3 pointB = simplexVector[idx0].b * u + simplexVector[idx1].b * v + simplexVector[idx2].b * w;

	float distance = glm::length(pointA - pointB);

	glm::vec3 normal = glm::normalize(pointA - pointB);

	CollisionPoints points;
	points.normal = normal;
	points.pointA = pointA;
	points.pointB = pointB;
	points.seperation = distance;

	std::cout << "contact point!!!!!!!!!!!!\n";
	std::cout << "contact A : " << pointA.x << " " << pointA.y << " " << pointA.z << "\n";
	std::cout << "contact B : " << pointB.x << " " << pointB.y << " " << pointB.z << "\n";
	std::cout << "contact normal : " << normal.x << " " << normal.y << " " << normal.z << "\n";
	std::cout << "seperation: " << points.seperation << "\n";
	pointsVector.push_back(points);

	return pointsVector;

	// seonjo 방법
	// std::vector<CollisionPoints> pointsVector;

	// std::cout << "dir: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";
	// std::cout << "seperation: " << minDistance << "\n";

	// std::vector<glm::vec4> candidatesA = getCandidates(boxA, minNormal);
	// std::vector<glm::vec4> candidatesB = getCandidates(boxB, -minNormal);

	// // std::cout << "candidateA start\n";
	// int32_t lengthA = candidatesA.size();
	// for (int32_t i = 0; i < lengthA; i++)
	// {
	// 	std::cout << "A judge\n";
	// 	CollisionPoints points;
	// 	glm::vec3 candidate = candidatesA[i];
	// 	float distance = candidatesA[i].w;
	// 	// std::cout << "distance: " << distance << "\n";
	// 	points.normal = minNormal;
	// 	points.seperation = minDistance - (candidatesA[0].w - distance);
	// 	float moveDistance = glm::dot(boxB.center - candidate, points.normal);
	// 	// std::cout << "moveDistance: " << moveDistance << "\n";
	// 	glm::vec3 movedPoint = candidate + moveDistance * points.normal;
	// 	// std::cout << "movePoint: " << movedPoint.x << " " << movedPoint.y << " " << movedPoint.z << "\n";
	// 	if (isContained(movedPoint, boxB, points.seperation, moveDistance))
	// 	{
	// 		std::cout << "AAAAAAAAAAA isContained!!\n";
	// 		std::cout << "candidate[" << i << "]: " << candidate.x << " " << candidate.y << " " << candidate.z << "\n";
	// 		std::cout << "seperation: " << points.seperation << "\n";
	// 		std::cout << "normal: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";

	// 		points.pointA = candidate;
	// 		points.pointB = candidate - points.normal * points.seperation;
	// 		pointsVector.push_back(points);
	// 	}
	// }

	// // std::cout << "candidateB start\n";

	// int32_t lengthB = candidatesB.size();
	// for (int32_t i = 0; i < lengthB; i++)
	// {
	// 	std::cout << "B judge\n";
	// 	CollisionPoints points;
	// 	glm::vec3 candidate = candidatesB[i];
	// 	float distance = candidatesB[i].w;
	// 	points.normal = minNormal;
	// 	points.seperation = minDistance - (candidatesB[0].w - distance);
	// 	float moveDistance = glm::dot(boxA.center - candidate, -points.normal);
	// 	glm::vec3 movedPoint = candidate + moveDistance * -points.normal;
	// 	if (isContained(movedPoint, boxA, points.seperation, moveDistance))
	// 	{
	// 		std::cout << "BBBBBBBBBBBB isContained!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
	// 		std::cout << "candidate[" << i << "]: " << candidate.x << " " << candidate.y << " " << candidate.z << "\n";
	// 		std::cout << "seperation: " << points.seperation << "\n";
	// 		std::cout << "normal: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";
	// 		points.pointB = candidate;
	// 		points.pointA = candidate + points.normal * points.seperation;
	// 		pointsVector.push_back(points);
	// 	}
	// }

	// return pointsVector;
}

void BoxToBoxContact::barycentric(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c, const glm::vec3 &p,
								  float &u, float &v, float &w)
{
	glm::vec3 v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = glm::dot(v0, v0);
	float d01 = glm::dot(v0, v1);
	float d11 = glm::dot(v1, v1);
	float d20 = glm::dot(v2, v0);
	float d21 = glm::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

bool BoxToBoxContact::isContained(const glm::vec3 &point, const BoxInfo &box, float seperation, float distance)
{
	glm::vec3 centerToPoint = point - box.center;

	if (seperation < distance)
	{
		return false;
	}

	for (int32_t i = 0; i < 3; i++)
	{
		const glm::vec3 &axis = box.axes[i];
		float projection = glm::dot(centerToPoint, axis);
		if (std::abs(projection) > box.halfSize[i])
		{
			return false;
		}
	}

	return true;
}

// simplex의 삼각형들의 법선벡터와 삼각형들중 원점에서 가장 멀리 떨어져있는 놈을 찾아서 반환
int32_t BoxToBoxContact::getFaceNormals(std::vector<glm::vec4> &normals, const std::vector<Simplex> &simplexVector,
										const std::vector<int32_t> &faces)
{
	int32_t minTriangle = 0;
	float minDistance = FLT_MAX;

	std::cout << "faces Size: " << faces.size() << "\n";
	// 삼각형 순회
	for (int32_t i = 0; i < faces.size(); i = i + 3)
	{
		// 삼각형 꼭짓점들
		glm::vec3 a = simplexVector[faces[i]].diff;
		glm::vec3 b = simplexVector[faces[i + 1]].diff;
		glm::vec3 c = simplexVector[faces[i + 2]].diff;

		// std::cout << "face[" << i << "]\n";
		// std::cout << "a: " << a.x << " " << a.y << " " << a.z << "\n";
		// std::cout << "b: " << b.x << " " << b.y << " " << b.z << "\n";
		// std::cout << "c: " << c.x << " " << c.y << " " << c.z << "\n";
		// 삼각형의 법선
		glm::vec3 crossResult = glm::cross(b - a, c - a);

		// face가 일직선일때 예외처리
		// if (glm::length2(crossResult) == 0)
		// {
		// 	faces.erase(faces.begin() + i);
		// 	faces.erase(faces.begin() + i);
		// 	faces.erase(faces.begin() + i);
		// 	i = i - 3;
		// 	continue;
		// }

		glm::vec3 normal = glm::normalize(crossResult);
		// 삼각형과 원점 사이의 거리
		float distance = glm::dot(normal, a);
		// 법선 방향이 반대인 경우 -1 곱해주기
		if (distance == 0)
		{
			int32_t simplexVectorSize = simplexVector.size();
			int32_t maxIdx = -1;
			float maxDistance = -FLT_MAX;
			for (int32_t j = 0; j < simplexVectorSize; j++)
			{
				float dotResult = glm::dot(simplexVector[j].diff, normal);
				if (maxDistance < dotResult)
				{
					maxDistance = dotResult;
					maxIdx = j;
				}
			}
			if (maxIdx != faces[i] && maxIdx != faces[i + 1] && maxIdx != faces[i + 2])
			{
				normal = -normal;
			}
		}
		else if (distance < 0)
		{
			normal *= -1;
			distance *= -1;
		}

		// std::cout << "face normal: " << normal.x << " " << normal.y << " " << normal.z << "\n";
		// std::cout << "face to o distance: " << distance << "\n";

		// 법선 벡터 저장
		normals.emplace_back(normal, distance);

		// 원점과 가장 가까운 삼각형 저장
		if (distance < minDistance)
		{
			minTriangle = i / 3;
			minDistance = distance;
		}
	}
	// std::cout << "minDistasnce: " << minDistance << "\n";
	// std::cout << "minFace: " << minTriangle << "\n";
	return minTriangle;
}

void BoxToBoxContact::addIfUniqueEdge(std::vector<std::pair<int32_t, int32_t>> &edges,
									  const std::vector<int32_t> &faces, int32_t a, int32_t b)
{
	auto reverse = std::find(			   //      0--<--3
		edges.begin(),					   //     / \ B /   A: 2-0
		edges.end(),					   //    / A \ /    B: 0-2
		std::make_pair(faces[b], faces[a]) //   1-->--2
	);

	if (reverse != edges.end())
	{
		edges.erase(reverse);
	}
	else
	{
		edges.emplace_back(faces[a], faces[b]);
	}
}

} // namespace ale