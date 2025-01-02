#include "physics/BoxToBoxContact.h"

namespace ale
{

BoxToBoxContact::BoxToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *BoxToBoxContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new BoxToBoxContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 BoxToBoxContact::supportA(const ConvexInfo &box, glm::vec3 dir)
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

glm::vec3 BoxToBoxContact::supportB(const ConvexInfo &box, glm::vec3 dir)
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

void BoxToBoxContact::findCollisionPoints(const ConvexInfo &boxA, const ConvexInfo &boxB,
										  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
										  std::vector<Simplex> &simplexVector)
{
	// clipping
	Face refFace = getPolygonFace(boxA, epaInfo.normal);
	Face incFace = getPolygonFace(boxB, -epaInfo.normal);

	std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);

	// 폴리곤의 각 꼭지점 -> 충돌점 여러 개
	buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
}

void BoxToBoxContact::buildManifoldFromPolygon(std::vector<CollisionInfo> &collisionInfoVector, const Face &refFace,
											   const Face &incFace, std::vector<glm::vec3> &polygon, EpaInfo &epaInfo)
{
	if (polygon.empty())
	{
		return;
	}

	// Incident 면의 plane 구하기 (간단히 incFace.normal, incFace.distance)
	// 혹은 실제로 dot(incFace.normal, incFace.vertices[0]) 등으로 distance를 구해도 됨
	glm::vec3 &normal = epaInfo.normal;
	float distance = epaInfo.distance;
	float refPlaneDist = refFace.distance;
	float incPlaneDist = incFace.distance;
	glm::vec3 refN = refFace.normal;
	glm::vec3 incN = incFace.normal;

	std::sort(polygon.begin(), polygon.end(),
			  [&refN](const glm::vec3 &a, const glm::vec3 &b) { return glm::dot(a, refN) < glm::dot(b, refN); });

	// 각 꼭지점마다 물체 A,B에서의 좌표를 구해 penetration 등 계산
	// 여기서는 "Ref Face plane에서 A 물체 좌표, Incident Face plane에서 B 물체 좌표" 라고 가정
	float denominator = (-(glm::dot(polygon[0], refN) - refPlaneDist));
	float ratio;
	if (denominator < 1e-8)
	{
		ratio = 0.0f;
	}
	else
	{
		ratio = distance / denominator;
	}

	for (const glm::vec3 &point : polygon)
	{
		// B측 point:
		glm::vec3 pointB = point;

		// 침투깊이
		float penentration = ratio * (-(glm::dot(point, refN) - refPlaneDist));

		// A측 point:
		glm::vec3 pointA = point + normal * penentration;

		// 접촉 정보
		CollisionInfo collisionInfo;
		collisionInfo.normal = refN;
		collisionInfo.pointA = pointA;
		collisionInfo.pointB = pointB;
		collisionInfo.seperation = penentration;
		collisionInfoVector.push_back(collisionInfo);
	}
}

std::vector<glm::vec3> BoxToBoxContact::clipPolygonAgainstPlane(const std::vector<glm::vec3> &polygon,
																const glm::vec3 &planeNormal, float planeDist)
{
	std::vector<glm::vec3> out;
	if (polygon.empty())
		return out;

	for (size_t i = 0; i < polygon.size(); i++)
	{
		const glm::vec3 &curr = polygon[i];
		const glm::vec3 &next = polygon[(i + 1) % polygon.size()];

		float distCurr = glm::dot(planeNormal, curr) - planeDist;
		float distNext = glm::dot(planeNormal, next) - planeDist;
		bool currInside = (distCurr <= 0.0f);
		bool nextInside = (distNext <= 0.0f);

		// CASE1: 둘 다 내부
		if (currInside && nextInside)
		{
			out.push_back(next);
		}
		// CASE2: 밖->안
		else if (!currInside && nextInside)
		{
			float t = distCurr / (distCurr - distNext);
			glm::vec3 intersect = curr + t * (next - curr);
			out.push_back(intersect);
			out.push_back(next);
		}
		// CASE3: 안->밖
		else if (currInside && !nextInside)
		{
			float t = distCurr / (distCurr - distNext);
			glm::vec3 intersect = curr + t * (next - curr);
			out.push_back(intersect);
		}
		// CASE4: 둘 다 밖 => nothing
	}

	return out;
}

std::vector<glm::vec3> BoxToBoxContact::computeContactPolygon(const Face &refFace, const Face &incFace)
{
	// 초기 polygon: Incident Face의 4점
	std::vector<glm::vec3> poly = incFace.vertices;

	// Ref Face의 4개 엣지로 만들어지는 '4개 사이드 평면'에 대해 클리핑
	// refFace.vertices = v0,v1,v2,v3 라고 가정, CCW 형태
	const std::vector<glm::vec3> &vertices = refFace.vertices;
	for (int32_t i = 0; i < 4; i++)
	{
		glm::vec3 start = vertices[i];
		glm::vec3 end = vertices[(i + 1) % 4];

		// edge
		glm::vec3 edge = end - start;

		// refFace.normal과 edge의 cross => 사이드 plane normal
		glm::vec3 sideN = glm::cross(refFace.normal, edge);
		sideN = glm::normalize(sideN);

		float planeDist = glm::dot(sideN, start);
		poly = clipPolygonAgainstPlane(poly, sideN, planeDist);

		if (poly.empty())
			break;
	}

	// 마지막으로 "Ref Face 자체" 평면에 대해서도 클리핑(뒤집힌 면 제거)
	poly = clipPolygonAgainstPlane(poly, refFace.normal, refFace.distance);

	return poly;
}

Face BoxToBoxContact::getPolygonFace(const ConvexInfo &box, const glm::vec3 &normal)
{
	// 1) box의 6개 면 중, worldNormal과 가장 유사한 면( dot > 0 ) 찾기
	// 2) 그 면의 4개 꼭지점을 구함
	// 여기서는 '법선이 박스의 +Y축과 가장 가까우면 top face' 식으로 단순화 예시
	// 실제 구현은 회전/transform 고려해야 함

	Face face;

	glm::vec3 axes[6] = {-box.axes[0], -box.axes[1], -box.axes[2], box.axes[0], box.axes[1], box.axes[2]};

	float maxDotRes = -FLT_MAX;
	int32_t maxIdx = -1;
	for (int32_t i = 0; i < 6; i++)
	{
		float nowDotRes = glm::dot(axes[i], normal);
		if (nowDotRes > maxDotRes)
		{
			maxDotRes = nowDotRes;
			maxIdx = i;
		}
	}

	glm::vec3 axis = axes[maxIdx];
	float centerDotRes = glm::dot(box.center, axis);
	glm::vec3 center(0.0f);

	for (const glm::vec3 point : box.points)
	{
		if (glm::dot(point, axis) > centerDotRes)
		{
			center += point;
			face.vertices.push_back(point);
		}
	}

	center = glm::vec3((center.x / face.vertices.size()), (center.y / face.vertices.size()),
					   (center.z / face.vertices.size()));
	face.normal = axis;
	face.distance = glm::dot(axis, face.vertices[0]);

	sortPointsClockwise(face.vertices, center, face.normal);

	return face;
}

void BoxToBoxContact::sortPointsClockwise(std::vector<glm::vec3> &points, const glm::vec3 &center,
										  const glm::vec3 &normal)
{
	// 1. 법선 벡터 기준으로 평면의 두 축 정의
	glm::vec3 u = glm::normalize(glm::cross(normal, glm::vec3(1.0f, 0.0f, 0.0f)));
	if (glm::length(u) < 1e-6)
	{ // normal이 x축과 평행한 경우 y축 사용
		u = glm::normalize(glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f)));
	}
	glm::vec3 v = glm::normalize(glm::cross(normal, u)); // 법선과 u의 외적

	// 2. 각도 계산 및 정렬
	auto angleComparator = [&center, &u, &v](const glm::vec3 &a, const glm::vec3 &b) {
		// a와 b를 u, v 축 기준으로 투영
		glm::vec3 da = a - center;
		glm::vec3 db = b - center;

		float angleA = atan2(glm::dot(da, v), glm::dot(da, u));
		float angleB = atan2(glm::dot(db, v), glm::dot(db, u));

		return angleA > angleB; // 시계 방향 정렬
	};
	std::sort(points.begin(), points.end(), angleComparator);
}

} // namespace ale