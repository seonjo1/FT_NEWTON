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
	glm::vec3 axisX = glm::normalize(boxA.points[1]- boxA.points[0]);
	glm::vec3 axisY = glm::normalize(boxA.points[2]- boxA.points[0]);
	glm::vec3 axisZ = glm::normalize(boxA.points[3]- boxA.points[0]);

	boxA.axes = {
		axisX, axisY, axisZ
	};

	// for (int k = 0; k < 8; k++)
	// {
	// 	std::cout << "pointA[" << k << "]: " << boxA.points[k].x << " " << boxA.points[k].y << " " << boxA.points[k].z
	// 			  << "\n";
	// }

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

	axisX = glm::normalize(boxB.points[1]- boxB.points[0]);
	axisY = glm::normalize(boxB.points[2]- boxB.points[0]);
	axisZ = glm::normalize(boxB.points[3]- boxB.points[0]);
	
	boxB.axes = {
		axisX, axisY, axisZ
	};


	// for (int k = 0; k < 8; k++)
	// {
	// 	std::cout << "pointB[" << k << "]: " << boxB.points[k].x << " " << boxB.points[k].y << " " << boxB.points[k].z
	// 			  << "\n";
	// }

	Simplex simplex;
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
	float dotAxes[3] =
	{
		glm::dot( box.axes[0], dir ) >= 0 ? 1.f : -1.f,
		glm::dot( box.axes[1], dir ) >= 0 ? 1.f : -1.f,
		glm::dot( box.axes[2], dir ) >= 0 ? 1.f : -1.f
	};

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
	
	std::sort(candidates.begin(), candidates.end(), [](const glm::vec4& v1, const glm::vec4& v2) {
        return v1.w > v2.w;
    });

	// for (int i = 0; i < candidates.size(); i++)
	// {
	// 	std::cout << "candidates[" << i << "]: " << candidates[i].x << " " << candidates[i].y << " " << candidates[i].z << "\n";
	// 	std::cout << "distance: " << candidates[i].w << "\n";
	// }

	return candidates;
}

glm::vec3 BoxToBoxContact::getSupportPoint(const BoxInfo &boxA, const BoxInfo &boxB, glm::vec3 &dir)
{
	std::cout << "dir: " << dir.x << " " << dir.y << " " << dir.z << "\n";
	glm::vec3 supA = supportBox(boxA, dir);
	glm::vec3 supB = supportBox(boxB, -dir);
	std::cout << "supportPoint: " << (supA - supB).x << " " << (supA - supB).y << " " << (supA - supB).z << "\n";
	return supA - supB;
}

bool BoxToBoxContact::handleLineSimplex(Simplex &simplex, glm::vec3 &dir)
{
	std::cout << "Line GJK\n";
	glm::vec3 &a = simplex.points[0];
	glm::vec3 &b = simplex.points[1];

	glm::vec3 ab = b - a;
	glm::vec3 ao = -a;

	dir = glm::normalize(glm::cross(glm::cross(ab, ao), ab));

	return false;
}

bool BoxToBoxContact::handleTriangleSimplex(Simplex &simplex, glm::vec3 &dir)
{
	std::cout << "Triangle GJK\n";
	glm::vec3 &a = simplex.points[0];
	glm::vec3 &b = simplex.points[1];
	glm::vec3 &c = simplex.points[2];

	glm::vec3 ab = b - a;
	glm::vec3 ac = c - a;
	glm::vec3 bc = c - b;
	glm::vec3 abc = glm::cross(ac, ab);

	dir = glm::normalize(glm::cross(abc, bc));
	if (glm::dot(dir, -c) > 0.0f)
	{
		// a 제거
		simplex.points.erase(simplex.points.begin());
		return false;
	}

	dir = glm::normalize(glm::cross(ac, abc));
	if (glm::dot(dir, -c) > 0.0f)
	{
		// b 제거
		simplex.points.erase(simplex.points.begin() + 1);
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

bool BoxToBoxContact::handleTetrahedronSimplex(Simplex &simplex, glm::vec3 &dir)
{
	std::cout << "Tetrahedron GJK\n";
	glm::vec3 &a = simplex.points[0];
	glm::vec3 &b = simplex.points[1];
	glm::vec3 &c = simplex.points[2];
	glm::vec3 &d = simplex.points[3];

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
		simplex.points.erase(simplex.points.begin());
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
		simplex.points.erase(simplex.points.begin() + 1);
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
		simplex.points.erase(simplex.points.begin() + 2);
		dir = glm::normalize(abd);
		return false;
	}

	return true;
}

// 가장 최근에 추가된 점 A = simplex.points.back() 로 가정
// simplex가 2, 3, 4개 점일 때 각각 처리 달라짐
bool BoxToBoxContact::handleSimplex(Simplex &simplex, glm::vec3 &dir)
{
	switch (simplex.points.size())
	{
	case 2:
		return handleLineSimplex(simplex, dir);
	case 3:
		return handleTriangleSimplex(simplex, dir);
	case 4:
		return handleTetrahedronSimplex(simplex, dir);
	}
	return false;
}

bool BoxToBoxContact::isDuplicatedPoint(const std::vector<glm::vec3> &points, const glm::vec3 &supportPoint)
{
	for (const glm::vec3 &point : points)
	{
		if (glm::length2(point - supportPoint) < 1e-6f)
		{
			return true;
		}
	}
	return false;
}

bool BoxToBoxContact::getGjkResult(const BoxInfo &boxA, const BoxInfo &boxB, Simplex &simplex)
{
	// 첫 번째 support point 구하기
	glm::vec3 dir = glm::normalize(boxB.center - boxA.center);
	if (glm::length2(dir) < 1e-8f)
	{
		dir = glm::vec3(1.0f, 0.0f, 0.0f);
	}

	glm::vec3 supportPoint = getSupportPoint(boxA, boxB, dir);
	simplex.points.push_back(supportPoint);

	// 두 번째 support point 구하기
	dir = -supportPoint;

	while (true)
	{
		// 새로운 서포트 점
		supportPoint = getSupportPoint(boxA, boxB, dir);

		// 만약 newSupport가 direction과 내적(dot)했을 때 0 이하라면
		// 더 이상 원점을 "방향 dir" 쪽에서 감쌀 수 없음 => 충돌X
		if (glm::dot(supportPoint, dir) < 0 || isDuplicatedPoint(simplex.points, supportPoint))
		{
			return false; // 교차하지 않음
		}

		// 심플렉스에 추가
		simplex.points.push_back(supportPoint);

		// 원점을 포함하는지 체크 및 simplex 갱신
		if (handleSimplex(simplex, dir))
		{
			// 원점 포함 => 충돌
			return true;
		}
	}

	return false;
}

bool BoxToBoxContact::isSameDirection(glm::vec3 v1, glm::vec3 v2)
{
	return glm::dot(v1, v2) > 0;
}

std::vector<CollisionPoints> BoxToBoxContact::getEpaResult(const BoxInfo &boxA, const BoxInfo &boxB, const Simplex &simplex)
{
	std::vector<glm::vec3> polytope(simplex.points.begin(), simplex.points.end());
	std::vector<int32_t> faces = {0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2};

	// std::cout << "polytope\n";
	// for (const glm::vec3& poly : polytope)
	// {
	// 	std::cout << poly.x << " " << poly.y << " " << poly.z << "\n";
	// }

	// GJK에서 구한 simplex들중 원점에서 가장 가까운 삼각형의 법선과 최소 거리
	std::vector<glm::vec4> normals;
	int32_t minFace = getFaceNormals(normals, polytope, faces);
	glm::vec3 minNormal;
	// std::cout << "minFace: " << minFace << "\n";
	float minDistance = FLT_MAX;

	while (minDistance == FLT_MAX)
	{
		// 최소 거리의 법선, 거리 쿼리
		minNormal = glm::vec3(normals[minFace]);
		minDistance = normals[minFace].w;

		// 최소 거리의 법선에 해당하는 supportPoint 쿼리
		glm::vec3 supportPoint = getSupportPoint(boxA, boxB, minNormal);

		// 원점에서 supportPoint까지의 거리
		float supportDistance = glm::dot(minNormal, supportPoint);

		// supportPoint가 현재 minDistance보다 원점에서 더 멀리있는 경우
		// 다시 원점에서부터 최소거리의 삼각형을 찾음
		if (std::abs(supportDistance - minDistance) > 1e-8f && !isDuplicatedPoint(polytope, supportPoint))
		{
			minDistance = FLT_MAX;
			std::vector<std::pair<int32_t, int32_t>> uniqueEdges;

			for (int32_t i = 0; i < normals.size(); i++)
			{
				// 법선 벡터와 supportPoint 방향의 벡터가 같은 방향인 경우
				if (isSameDirection(normals[i], supportPoint))
				{
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

			// uniqueEdge에 있는 edge들과 새로운 점을 조함하여 face 생성
			std::vector<int32_t> newFaces;
			for (auto [edgeIndex1, edgeIndex2] : uniqueEdges)
			{
				newFaces.push_back(edgeIndex1);
				newFaces.push_back(edgeIndex2);
				newFaces.push_back(polytope.size());
			}

			// 새로운 점 추가
			polytope.push_back(supportPoint);

			// 새로운 삼각형의 normal 벡터들과 최소 거리 쿼리
			std::vector<glm::vec4> newNormals;
			int32_t newMinFace = getFaceNormals(newNormals, polytope, newFaces);
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
			faces.insert(faces.end(), newFaces.begin(), newFaces.end());
			normals.insert(normals.end(), newNormals.begin(), newNormals.end());
		}
	}

	std::vector<CollisionPoints> pointsVector;

	std::cout << "dir: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";
	// points.normal = minNormal;
	// points.seperation = minDistance;
	std::vector<glm::vec4> candidatesA = getCandidates(boxA, minNormal);
	std::vector<glm::vec4> candidatesB = getCandidates(boxB, -minNormal);

	std::cout << "candidateA start\n";
	bool aCollision = false;
	int32_t lengthA = candidatesA.size();
	for (int32_t i = 0; i < lengthA; i++)
	{
		CollisionPoints points;
		glm::vec3 candidate = candidatesA[i];
		float distance = candidatesA[i].w;
		// std::cout << "distance: " << distance << "\n";
		points.normal = minNormal;
		points.seperation = minDistance - (candidatesA[0].w - distance);
		float moveDistance = glm::dot(boxB.center - candidate, points.normal);
		// std::cout << "moveDistance: " << moveDistance << "\n";
		glm::vec3 movedPoint = candidate + moveDistance * points.normal;
		// std::cout << "movePoint: " << movedPoint.x << " " << movedPoint.y << " " << movedPoint.z << "\n";
		if (isContained(movedPoint, boxB, points.seperation, moveDistance))
		{
			// std::cout << "AAAAAAAAAAA isContained!!\n";
			// std::cout << "candidate[" << i << "]: " << candidate.x << " " << candidate.y << " " << candidate.z << "\n";
			// std::cout << "seperation: " << points.seperation << "\n";
			// std::cout << "normal: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";

			points.pointA = candidate;
			points.pointB = candidate - points.normal * points.seperation;
			aCollision = true;
			pointsVector.push_back(points);
		}
	}

	if (aCollision)
	{
		return pointsVector;
	}

	// std::cout << "candidateB start\n";

	int32_t lengthB = candidatesB.size();
	for (int32_t i = 0; i < lengthB; i++)
	{
		CollisionPoints points;
		glm::vec3 candidate = candidatesB[i];
		float distance = candidatesB[i].w;
		points.normal = minNormal;
		points.seperation = minDistance - (candidatesB[0].w - distance);
		float moveDistance = glm::dot(boxA.center - candidate, -points.normal);
		glm::vec3 movedPoint = candidate + moveDistance * -points.normal;
		if (isContained(movedPoint, boxA, points.seperation, moveDistance))
		{
			// std::cout << "BBBBBBBBBBBB isContained!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";	
			// std::cout << "candidate[" << i << "]: " << candidate.x << " " << candidate.y << " " << candidate.z << "\n";
			// std::cout << "seperation: " << points.seperation << "\n";
			// std::cout << "normal: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";
			points.pointB = candidate;
			points.pointA = candidate + points.normal * points.seperation;
			pointsVector.push_back(points);
		}
	}

	return pointsVector;
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
int32_t BoxToBoxContact::getFaceNormals(std::vector<glm::vec4> &normals, const std::vector<glm::vec3> &polytope,
										const std::vector<int32_t> &faces)
{
	int32_t minTriangle = 0;
	float minDistance = FLT_MAX;

	// 삼각형 순회
	for (int32_t i = 0; i < faces.size(); i = i + 3)
	{
		// 삼각형 꼭짓점들
		glm::vec3 a = polytope[faces[i]];
		glm::vec3 b = polytope[faces[i + 1]];
		glm::vec3 c = polytope[faces[i + 2]];

		// std::cout << "face[" << i << "]\n";
		// std::cout << "a: " << a.x << " " << a.y << " " << a.z << "\n";
		// std::cout << "b: " << b.x << " " << b.y << " " << b.z << "\n";
		// std::cout << "c: " << c.x << " " << c.y << " " << c.z << "\n";
		// 삼각형의 법선
		glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
		// 삼각형과 원점 사이의 거리
		float distance = glm::dot(normal, a);
		// 법선 방향이 반대인 경우 -1 곱해주기
		if (distance <= 0)
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