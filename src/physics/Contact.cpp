#include "physics/Contact.h"
#include "physics/ShapeCollisions.h"

namespace ale
{

int32_t operator&(int32_t val, EContactFlag flag)
{
	return val & static_cast<int32_t>(flag);
}

int32_t operator|(int32_t val, EContactFlag flag)
{
	return val | static_cast<int32_t>(flag);
}

int32_t operator~(EContactFlag flag)
{
	return ~static_cast<int32_t>(flag);
}

bool operator==(int32_t val, EContactFlag flag)
{
	return static_cast<int32_t>(flag) == val;
}

contactMemberFunction Contact::createContactFunctions[32] = {
	nullptr,							// 0
	&SphereToSphereContact::create,		// 01
	&BoxToBoxContact::create,			// 10
	&SphereToBoxContact::create,		// 11
	&BoxToBoxContact::create,			// 100
	&SphereToBoxContact::create,		// 101
	&BoxToBoxContact::create,			// 110
	nullptr,							// 111
	&CylinderToCylinderContact::create, // 1000
	&SphereToCylinderContact::create,	// 1001
	&BoxToCylinderContact::create,		// 1010
	nullptr,							// 1011
	&BoxToCylinderContact::create,		// 1100
	nullptr,							// 1101
	nullptr,							// 1110
	nullptr,							// 1111
	&CapsuleToCapsuleContact::create,	// 10000
	&SphereToCapsuleContact::create,	// 10001
	&BoxToCapsuleContact::create,		// 10010
	nullptr,							// 10011
	&BoxToCapsuleContact::create,		// 10100
	nullptr,							// 10101
	nullptr,							// 10110
	nullptr,							// 10111
	&CylinderToCapsuleContact::create,	// 11000
	nullptr,							// 11001
	nullptr,							// 11010
	nullptr,							// 11011
	nullptr,							// 11100
	nullptr,							// 11101
	nullptr,							// 11110
	nullptr,							// 11111
};

Contact::Contact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: m_fixtureA(fixtureA), m_fixtureB(fixtureB), m_indexA(indexA), m_indexB(indexB)
{
	m_flags = static_cast<int32_t>(EContactFlag::TOUCHING);

	m_fixtureA = fixtureA;
	m_fixtureB = fixtureB;

	m_indexA = indexA;
	m_indexB = indexB;

	m_prev = nullptr;
	m_next = nullptr;

	m_nodeA.contact = nullptr;
	m_nodeA.prev = nullptr;
	m_nodeA.next = nullptr;
	m_nodeA.other = nullptr;

	m_nodeB.contact = nullptr;
	m_nodeB.prev = nullptr;
	m_nodeB.next = nullptr;
	m_nodeB.other = nullptr;

	m_friction = std::sqrt(m_fixtureA->getFriction() * m_fixtureB->getFriction());
	m_restitution = std::max(m_fixtureA->getRestitution(), m_fixtureB->getRestitution());
}

Contact *Contact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	// 각 fixture의 shape의 type 가져오기
	EType type1 = fixtureA->getType();
	EType type2 = fixtureB->getType();

	if (type1 > type2)
	{
		Fixture *tmpFixture = fixtureA;
		int32_t tmpIndex = indexA;
		fixtureA = fixtureB;
		indexA = indexB;
		fixtureB = tmpFixture;
		indexB = tmpIndex;
	}

	return createContactFunctions[type1 | type2](fixtureA, fixtureB, indexA, indexB);
}

void Contact::evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB)
{
	// std::cout << "\n\n\n\n\nevaluate start\n";
	Shape *shapeA = m_fixtureA->getShape();
	Shape *shapeB = m_fixtureB->getShape();

	// std::cout << "bodyA: " << m_fixtureA->getBody()->getBodyId() << "\n";
	// std::cout << "bodyB: " << m_fixtureB->getBody()->getBodyId() << "\n";

	// std::cout << "getShapeInfo!!\n";
	ConvexInfo convexA = shapeA->getShapeInfo(transformA);
	ConvexInfo convexB = shapeB->getShapeInfo(transformB);

	SimplexArray simplexArray;
	CollisionInfo collisionInfo;

	simplexArray.simplexCount = 0;
	collisionInfo.size = 0;

	// Sphere to Sphere Collide
	if (shapeA->getType() == EType::SPHERE && shapeB->getType() == EType::SPHERE)
	{
		if (checkSphereToSphereCollide(convexA, convexB) == true)
		{
			EpaInfo epaInfo;
			glm::vec3 centerDistance = convexB.center - convexA.center;
			epaInfo.normal = glm::normalize(centerDistance);
			epaInfo.distance = convexA.radius + convexB.radius - glm::length(centerDistance);

			// std::cout << "CLIPPING start\n";
			findCollisionPoints(convexA, convexB, collisionInfo, epaInfo, simplexArray);

			// std::cout << "createManifold start\n";
			generateManifolds(collisionInfo, manifold, m_fixtureA, m_fixtureB);
		}
		return;
	}

	// std::cout << "GJK start\n";
	bool isCollide = getGjkResult(convexA, convexB, simplexArray);

	if (isCollide)
	{
		// std::cout << "EPA start\n";
		EpaInfo epaInfo = getEpaResult(convexA, convexB, simplexArray);

		if (epaInfo.distance == -1.0f)
		{
			return;
		}

		// std::cout << "CLIPPING start\n";
		findCollisionPoints(convexA, convexB, collisionInfo, epaInfo, simplexArray);

		// std::cout << "createManifold start\n";
		generateManifolds(collisionInfo, manifold, m_fixtureA, m_fixtureB);
	}
}

void Contact::update()
{
	// 기존 manifold 저장
	// Manifold oldManifold = m_manifold;

	// 이전 프레임에서 두 객체가 충돌중이었는지 확인
	bool touching = false;

	// bodyA, bodyB의 Transform 가져오기
	Rigidbody *bodyA = m_fixtureA->getBody();
	Rigidbody *bodyB = m_fixtureB->getBody();
	const Transform &transformA = bodyA->getTransform();
	const Transform &transformB = bodyB->getTransform();

	// std::cout << "collide bodyA: " << bodyA->getBodyId() << " bodyB: " << bodyB->getBodyId() << "\n";

	// Evaluate
	// 두 shape의 변환 상태를 적용해 world space에서의 충돌 정보를 계산
	// 1. 두 도형이 실제로 충돌하는지 검사
	// 2. 충돌에 따른 manifold 생성
	// 3. manifold의 내부 값을 impulse를 제외하고 채워줌
	// 4. 실제 충돌이 일어나지 않은 경우 manifold.pointCount = 0인 충돌 생성
	m_manifold.pointsCount = 0;
	// std::cout << "start evaluate!!\n";
	evaluate(m_manifold, transformA, transformB);
	// std::cout << "finish evaluate!!\n";
	touching = m_manifold.pointsCount > 0;

	// manifold의 충격량 0으로 초기화 및 old manifold 중
	// 같은 충돌이 있는경우 Impulse 재사용
	// id 는 충돌 도형의 type과 vertex 또는 line의 index 정보를 압축하여 결정

	// warm start
	// for (ManifoldPoint &manifoldPoint : m_manifold.points)
	// {

	// 	manifoldPoint.normalImpulse = 0.0f;
	// 	manifoldPoint.tangentImpulse = 0.0f;
	// 	uint32_t manifoldPointId = manifoldPoint.id;

	// 	for (ManifoldPoint &oldManifoldPoint : oldManifold.points)
	// 	{
	// 		// oldmanifold에 똑같은 manifold가 존재하는 경우 impulse 덮어쓰기
	// 		if (oldManifoldPoint.id == manifoldPointId)
	// 		{
	// 			manifoldPoint.normalImpulse = oldManifoldPoint.normalImpulse;
	// 			manifoldPoint.tangentImpulse = oldManifoldPoint.tangentImpulse;
	// 			break;
	// 		}
	// 	}
	// }

	if (touching)
	{
		m_flags = m_flags | EContactFlag::TOUCHING;
	}
	else
	{
		m_flags = m_flags & ~EContactFlag::TOUCHING;
	}
}

float Contact::getFriction() const
{
	return m_friction;
}

float Contact::getRestitution() const
{
	return m_restitution;
}

Contact *Contact::getNext()
{
	return m_next;
}

Fixture *Contact::getFixtureA() const
{
	return m_fixtureA;
}

Fixture *Contact::getFixtureB() const
{
	return m_fixtureB;
}

int32_t Contact::getChildIndexA() const
{
	return m_indexA;
}

int32_t Contact::getChildIndexB() const
{
	return m_indexB;
}

ContactLink *Contact::getNodeA()
{
	return &m_nodeA;
}

ContactLink *Contact::getNodeB()
{
	return &m_nodeB;
}

Manifold &Contact::getManifold()
{
	return m_manifold;
}

void Contact::setPrev(Contact *contact)
{
	m_prev = contact;
}

void Contact::setNext(Contact *contact)
{
	m_next = contact;
}

void Contact::setFlag(EContactFlag flag)
{
	m_flags = m_flags | static_cast<int32_t>(flag);
}

void Contact::unsetFlag(EContactFlag flag)
{
	m_flags = m_flags & ~static_cast<int32_t>(flag);
}

bool Contact::hasFlag(EContactFlag flag)
{
	return (m_flags & static_cast<int32_t>(flag)) == static_cast<int32_t>(flag);
}

// manifold functions

Simplex Contact::getSupportPoint(const ConvexInfo &convexA, const ConvexInfo &convexB, glm::vec3 &dir)
{
	Simplex simplex;
	// std::cout << "getSupportPoint start!!\n";
	// std::cout << "dir: " << dir.x << " " << dir.y << " " << dir.z << "\n";
	simplex.a = supportA(convexA, dir);
	simplex.b = supportB(convexB, -dir);
	simplex.diff = simplex.a - simplex.b;

	// std::cout << "simplex.a : " << simplex.a.x << " " << simplex.a.y << " " << simplex.a.z << "\n";
	// std::cout << "simplex.b : " << simplex.b.x << " " << simplex.b.y << " " << simplex.b.z << "\n";
	// std::cout << "simplex.diff : " << simplex.diff.x << " " << simplex.diff.y << " " << simplex.diff.z << "\n";
	// std::cout << "getSupportPoint end!!\n";
	return simplex;
}

bool Contact::handleLineSimplex(SimplexArray &simplexArray, glm::vec3 &dir)
{
	// std::cout << "Line GJK\n";
	glm::vec3 &a = simplexArray.simplices[0].diff;
	glm::vec3 &b = simplexArray.simplices[1].diff;

	glm::vec3 ab = b - a;
	glm::vec3 ao = -a;

	if (isSameDirection(ab, ao))
	{
		// std::cout << "sameDirection!!\n";
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

	// std::cout << "line normal: " << dir.x << " " << dir.y << " " << dir.z << "\n";
	return false;
}

bool Contact::handleTriangleSimplex(SimplexArray &simplexArray, glm::vec3 &dir)
{
	// std::cout << "Triangle GJK\n";
	glm::vec3 &a = simplexArray.simplices[0].diff;
	glm::vec3 &b = simplexArray.simplices[1].diff;
	glm::vec3 &c = simplexArray.simplices[2].diff;

	glm::vec3 ab = b - a;
	glm::vec3 ac = c - a;
	glm::vec3 bc = c - b;
	glm::vec3 abc = glm::cross(ac, ab);

	// 면적이 없으면 반대 dir로 세 번째 점 다시 찾기기
	if (glm::length2(abc) == 0.0f)
	{
		--simplexArray.simplexCount;
		dir = -dir;
		return false;
	}

	dir = glm::normalize(glm::cross(abc, bc));
	if (glm::dot(dir, -c) > 0.0f)
	{
		// a 제거
		simplexArray.simplices[0] = simplexArray.simplices[1];
		simplexArray.simplices[1] = simplexArray.simplices[2];
		--simplexArray.simplexCount;
		return false;
	}

	dir = glm::normalize(glm::cross(ac, abc));
	if (glm::dot(dir, -c) > 0.0f)
	{
		// b 제거
		simplexArray.simplices[1] = simplexArray.simplices[2];
		--simplexArray.simplexCount;
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

bool Contact::handleTetrahedronSimplex(SimplexArray &simplexArray, glm::vec3 &dir)
{
	// std::cout << "Tetrahedron GJK\n";
	glm::vec3 &a = simplexArray.simplices[0].diff;
	glm::vec3 &b = simplexArray.simplices[1].diff;
	glm::vec3 &c = simplexArray.simplices[2].diff;
	glm::vec3 &d = simplexArray.simplices[3].diff;

	glm::vec3 abc = glm::cross((b - a), (c - a));
	// 부피가 없으면 반대 방향으로 4번째 점 다시 찾기기
	if (std::abs(glm::dot((d - a), abc)) < 1e-8f)
	{
		// std::cout << "no Volume!!\n";
		--simplexArray.simplexCount;
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
		simplexArray.simplices[0] = simplexArray.simplices[1];
		simplexArray.simplices[1] = simplexArray.simplices[2];
		simplexArray.simplices[2] = simplexArray.simplices[3];
		--simplexArray.simplexCount;
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
		simplexArray.simplices[1] = simplexArray.simplices[2];
		simplexArray.simplices[2] = simplexArray.simplices[3];
		--simplexArray.simplexCount;
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
		simplexArray.simplices[2] = simplexArray.simplices[3];
		--simplexArray.simplexCount;
		dir = glm::normalize(abd);
		return false;
	}

	return true;
}

// 가장 최근에 추가된 점 A = simplex.points.back() 로 가정
// simplex가 2, 3, 4개 점일 때 각각 처리 달라짐
bool Contact::handleSimplex(SimplexArray &simplexArray, glm::vec3 &dir)
{
	switch (simplexArray.simplexCount)
	{
	case 2:
		return handleLineSimplex(simplexArray, dir);
	case 3:
		return handleTriangleSimplex(simplexArray, dir);
	case 4:
		return handleTetrahedronSimplex(simplexArray, dir);
	}
	return false;
}

bool Contact::isDuplicatedPoint(const SimplexArray &simplexArray, const glm::vec3 &supportPoint)
{
	int32_t size = simplexArray.simplexCount;
	for (int32_t i = 0; i < size; i++)
	{
		if (glm::length2(simplexArray.simplices[i].diff - supportPoint) < 1e-6f)
		{
			return true;
		}
	}
	return false;
}

bool Contact::checkSphereToSphereCollide(const ConvexInfo &convexA, const ConvexInfo &convexB)
{
	if (glm::length(convexA.center - convexB.center) < (convexA.radius + convexB.radius))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Contact::getGjkResult(const ConvexInfo &convexA, const ConvexInfo &convexB, SimplexArray &simplexArray)
{
	const int32_t ITERATION = 64;

	// 첫 번째 support point 구하기
	glm::vec3 dir = glm::normalize(convexB.center - convexA.center);
	if (glm::length2(dir) < 1e-8f)
	{
		dir = glm::vec3(1.0f, 0.0f, 0.0f);
	}

	simplexArray.simplices[0] = getSupportPoint(convexA, convexB, dir);
	++simplexArray.simplexCount;

	glm::vec3 supportPoint = simplexArray.simplices[0].diff;

	// 두 번째 support point 구하기
	if (glm::length2(supportPoint) == 0.0f)
	{
		dir = -dir;
		simplexArray.simplices[0] = getSupportPoint(convexA, convexB, dir);
		supportPoint = simplexArray.simplices[0].diff;
	}

	dir = glm::normalize(-supportPoint);

	int32_t iter = 0;
	while (iter < ITERATION)
	{
		// 새로운 서포트 점
		Simplex simplex = getSupportPoint(convexA, convexB, dir);
		supportPoint = simplex.diff;

		// 만약 newSupport가 direction과 내적(dot)했을 때 0 이하라면
		// 더 이상 원점을 "방향 dir" 쪽에서 감쌀 수 없음 => 충돌X
		if (glm::dot(supportPoint, dir) < 0 || isDuplicatedPoint(simplexArray, supportPoint))
		{
			return false; // 교차하지 않음
		}

		// 심플렉스에 추가
		simplexArray.simplices[simplexArray.simplexCount] = simplex;
		++simplexArray.simplexCount;

		// 원점을 포함하는지 체크 및 simplex 갱신
		if (handleSimplex(simplexArray, dir))
		{
			// 원점 포함 => 충돌
			return true;
		}
		iter++;
	}

	return false;
}

bool Contact::isSimilarDirection(glm::vec3 v1, glm::vec3 v2)
{
	return glm::dot(v1, v2) > 0.0f;
}

bool Contact::isSameDirection(glm::vec3 v1, glm::vec3 v2)
{
	return glm::length2(glm::cross(v1, v2)) == 0.0f;
}

EpaInfo Contact::getEpaResult(const ConvexInfo &convexA, const ConvexInfo &convexB, SimplexArray &simplexArray)
{
	FaceArray faceArray;
	FaceArray newFaceArray;
	int32_t initIdx[12] = {0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2};

	memcpy(faceArray.faces, initIdx, sizeof(int32_t) * 12);
	faceArray.count = 4;

	// std::cout << "faceAttray.count: " << faceArray.count << "\n";
	// GJK에서 구한 simplex들중 원점에서 가장 가까운 삼각형의 법선과 최소 거리
	int32_t minFace = getFaceNormals(simplexArray, faceArray);
	glm::vec3 minNormal(0.0f);
	float minDistance = FLT_MAX;

	if (minFace == -1)
	{
		minDistance = -1.0f;
	}
	// std::cout << "faceAttray.count: " << faceArray.count << "\n";

	// int b = 1;
	while (minDistance == FLT_MAX)
	{
		// 	std::cout << "b: " << b << "\n";
		// 	b++;
		// std::cout << "simplex!!\n";
		// for (Simplex &simplex : simplexVector)
		// {
		// 	std::cout << "(" << simplex.diff.x << ", " << simplex.diff.y << ", " << simplex.diff.z << ")\n";
		// }

		// std::cout << "loop start\n";
		// std::cout << "faceAttray.count: " << faceArray.count << "\n";

		// 최소 거리의 법선, 거리 쿼리
		minNormal = glm::vec3(faceArray.normals[minFace]);
		minDistance = faceArray.normals[minFace].w;

		// std::cout << "minNormal: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";
		// std::cout << "minDistance: " << minDistance << "\n";

		// 최소 거리의 법선에 해당하는 supportPoint 쿼리

		Simplex simplex = getSupportPoint(convexA, convexB, minNormal);
		glm::vec3 supportPoint = simplex.diff;

		// 원점에서 supportPoint까지의 거리
		float supportDistance = glm::dot(minNormal, supportPoint);

		// supportPoint가 현재 minDistance보다 원점에서 더 멀리있는 경우
		// 다시 원점에서부터 최소거리의 삼각형을 찾음
		// std::cout << "supportDistance: " << supportDistance << "\n";
		// std::cout << "minDistance: " << minDistance << "\n";
		// std::cout << "faceAttray.count: " << faceArray.count << "\n";

		if (std::abs(supportDistance - minDistance) > 1e-2f && !isDuplicatedPoint(simplexArray, supportPoint))
		{
			// std::cout << "faceAttray.count: " << faceArray.count << "\n";
			minDistance = FLT_MAX;

			UniqueEdges uniqueEdges;
			void *memory = PhysicsAllocator::m_stackAllocator.allocateStack(sizeof(std::pair<int32_t, int32_t>) *
																			faceArray.count * 3);
			uniqueEdges.edges = static_cast<std::pair<int32_t, int32_t> *>(memory);
			uniqueEdges.size = 0;

			for (int32_t i = 0; i < faceArray.count; i++)
			{
				// std::cout << "start addUniqueEdge()\n";
				glm::vec3 center = (simplexArray.simplices[faceArray.faces[i * 3]].diff +
									simplexArray.simplices[faceArray.faces[i * 3 + 1]].diff +
									simplexArray.simplices[faceArray.faces[i * 3 + 2]].diff) /
								   3.0f;
				// std::cout << "faceAttray.count: " << faceArray.count << "\n";
				// std::cout << "center: " << center.x << " " << center.y << " " << center.z << "\n";
				// std::cout << "supportPoint: (" << supportPoint.x << ", " << supportPoint.y << ", " << supportPoint.z
				// << ")\n"; std::cout << "supportPoint - center: " << supportPoint.x - center.x << " " <<
				// supportPoint.y - center.y << " " << supportPoint.z - center.z << "\n"; std::cout << "normals[" << i
				// << "]: " << normals[i].x << " " << normals[i].y << " " << normals[i].z << "\n";
				if (isSimilarDirection(faceArray.normals[i], supportPoint - center))
				{
					int32_t faceIdx = i * 3;
					// std::cout << "face " << i << " is in!!!!!!!!\n";
					// std::cout << "faces: " << faces[faceIdx] << " " << faces[faceIdx + 1] << " " << faces[faceIdx +
					// 2] << "\n";

					// 해당 법선의 기존 삼각형의 edge들을 uniqueEdges에 저장
					// 만약 같은 edge가 2번 들어오면 사라질 edge로 판단하여 삭제
					// 1번만 들어오는 edge들만 모아서 새로운 점과 조합하여 새로운 삼각형 생성성
					addIfUniqueEdge(uniqueEdges, faceArray.faces, faceIdx, faceIdx + 1);
					addIfUniqueEdge(uniqueEdges, faceArray.faces, faceIdx + 1, faceIdx + 2);
					addIfUniqueEdge(uniqueEdges, faceArray.faces, faceIdx + 2, faceIdx);

					int32_t normalLastIdx = faceArray.count - 1;
					int32_t faceLastIdx = (normalLastIdx) * 3;
					// std::cout << "normalLastIdx: " << normalLastIdx << "\n";
					// std::cout << "faceLastIdx: " << faceLastIdx << "\n";
					faceArray.faces[faceIdx] = faceArray.faces[faceLastIdx];
					faceArray.faces[faceIdx + 1] = faceArray.faces[faceLastIdx + 1];
					faceArray.faces[faceIdx + 2] = faceArray.faces[faceLastIdx + 2];
					faceArray.normals[i] = faceArray.normals[normalLastIdx]; // pop-erase

					--faceArray.count;

					--i;
				}
			}

			// uniqueEdge에 있는 edge들과 새로운 점을 조함하여 face 생성

			newFaceArray.count = 0;

			int32_t uniqueEdgesCount = uniqueEdges.size;
			for (int32_t i = 0; i < uniqueEdgesCount; ++i)
			{
				int32_t edgeIdx1 = uniqueEdges.edges[i].first;
				int32_t edgeIdx2 = uniqueEdges.edges[i].second;
				// std::cout << "add uniqueEdgeIdx1: " << edgeIndex1 << "\n";
				// std::cout << "add uniqueEdgeIdx2: " << edgeIndex2 << "\n";
				addFaceInFaceArray(newFaceArray, edgeIdx1, edgeIdx2, simplexArray.simplexCount);
			}

			// 새로 추가되는 면이 없다면 종료료
			if (newFaceArray.count == 0)
			{
				// std::cout << "simplex!!\n";
				// for (Simplex &simplex : simplexVector)
				// {
				// 	std::cout << "(" << simplex.diff.x << ", " << simplex.diff.y << ", " << simplex.diff.z << ")\n";
				// }

				// std::cout << "normals size: " << normals.size() << "\n";
				// std::cout << "minNormal: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";

				throw std::runtime_error("failed to EPA!");
			}

			PhysicsAllocator::m_stackAllocator.freeStack();

			// std::cout << "add simplex!!\n";

			// 새로운 점 추가
			simplexArray.simplices[simplexArray.simplexCount] = simplex;
			++simplexArray.simplexCount;

			// 새로운 삼각형의 normal 벡터들과 최소 거리 쿼리
			int32_t newMinFace = getFaceNormals(simplexArray, newFaceArray);
			float oldMinDistance = FLT_MAX;

			if (newMinFace == -1)
			{
				minNormal = glm::vec3(0.0f);
				minDistance = -1.0f;
				break;
			}

			// std::cout << "find min dist\n";
			// 기존 삼각형들 중 가장 거리가 짧은 삼각형 쿼리
			int32_t normalsSize = faceArray.count;
			for (int32_t i = 0; i < normalsSize; i++)
			{
				if (faceArray.normals[i].w < oldMinDistance)
				{
					oldMinDistance = faceArray.normals[i].w;
					minFace = i;
				}
			}

			// std::cout << "find new min dist\n";

			// 새로운 삼각형들중 가장 짧은 거리가 존재하면 해당 face를 minFace로 설정
			if (newFaceArray.normals[newMinFace].w < oldMinDistance)
			{
				minFace = newMinFace + faceArray.count;
			}

			// std::cout << "merge!!\n";
			// 새로운 face, normal 추가
			mergeFaceArray(faceArray, newFaceArray);
			// for (int z = 0; z < faces.size(); z = z + 3)
			// {
			// 	std::cout << "face: " << faces[z] << " " << faces[z + 1] << " " << faces[z + 2] << "\n";
			// }
		}
	}

	EpaInfo epaInfo;
	epaInfo.normal = minNormal;
	epaInfo.distance = minDistance;

	// std::cout << "minNormal: " << minNormal.x << " " << minNormal.y << " " << minNormal.z << "\n";
	// std::cout << "minDistance: " << minDistance << "\n";

	// std::cout << "epa end!!\n";
	return epaInfo;
}

// simplex의 삼각형들의 법선벡터와 삼각형들중 원점에서 가장 멀리 떨어져있는 놈을 찾아서 반환
int32_t Contact::getFaceNormals(SimplexArray &simplexArray, FaceArray &faceArray)
{
	glm::vec3 center(0.0f);

	int32_t simplexSize = simplexArray.simplexCount;
	for (int32_t i = 0; i < simplexSize; ++i)
	{
		center += simplexArray.simplices[i].diff;
	}

	center = center / static_cast<float>(simplexSize);

	// std::cout << " get Face Normals center : (" << center.x << ", " << center.y << ", " << center.z << ")\n";

	int32_t minTriangle = 0;
	float minDistance = FLT_MAX;

	// std::cout << "faces Size: " << faces.size() << "\n";
	// 삼각형 순회
	int32_t facesSize = faceArray.count * 3;
	for (int32_t i = 0; i < facesSize; i = i + 3)
	{
		// 삼각형 꼭짓점들
		glm::vec3 &a = simplexArray.simplices[faceArray.faces[i]].diff;
		glm::vec3 &b = simplexArray.simplices[faceArray.faces[i + 1]].diff;
		glm::vec3 &c = simplexArray.simplices[faceArray.faces[i + 2]].diff;

		// std::cout << "pointA: " << a.x << " " << a.y << " " << a.z << "\n";
		// std::cout << "pointB: " << b.x << " " << b.y << " " << b.z << "\n";
		// std::cout << "pointC: " << c.x << " " << c.y << " " << c.z << "\n";

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
		// std::cout  << "before normal: " << normal.x << " " << normal.y << " " << normal.z << "\n";
		// 삼각형과 원점 사이의 거리
		if (glm::dot(normal, a - center) < 0)
		{
			normal = -normal;
		}

		float distance = glm::dot(normal, a);
		if (distance < 0)
		{
			return -1;
		}

		// 법선 벡터 저장
		faceArray.normals[i / 3] = glm::vec4(normal, distance);

		// 원점과 가장 가까운 삼각형 저장
		if (distance < minDistance)
		{
			minTriangle = i / 3;
			minDistance = distance;
		}
	}

	return minTriangle;
}

void Contact::addIfUniqueEdge(UniqueEdges &uniqueEdges, const int32_t *faces, int32_t p1, int32_t p2)
{
	int32_t size = uniqueEdges.size;
	int32_t i = 0;
	bool removeEdge = false;
	for (; i < size; ++i)
	{
		std::pair<int32_t, int32_t> &edge = uniqueEdges.edges[i];
		if (edge.first == faces[p2] && edge.second == faces[p1])
		{
			removeEdge = true;
			++i;
			break ;
		}
	}

	for (; i < size; ++i)
	{
		uniqueEdges.edges[i - 1] = uniqueEdges.edges[i];
	}

	if (removeEdge)
	{
		--uniqueEdges.size;
	}
	else
	{
		uniqueEdges.edges[uniqueEdges.size] = std::make_pair(faces[p1], faces[p2]);
		++uniqueEdges.size;
	}
}

void Contact::generateManifolds(CollisionInfo &collisionInfo, Manifold &manifold, Fixture *m_fixtureA,
								Fixture *m_fixtureB)
{
	/*
		64bit = 27bit(small proxyId) 	 |
				27bit(big proxyId) 		 |
				5bit(small contact part) |
				5bit(big contact part)
	*/
	// static int64_t bitmask = 0xFFFFFFFF & ~0b11111;

	int32_t collisionInfoSize = collisionInfo.size;
	manifold.pointsCount = collisionInfoSize;

	for (int32_t i = 0; i < collisionInfoSize; ++i)
	{
		manifold.points[i].pointA = collisionInfo.pointA[i];
		manifold.points[i].pointB = collisionInfo.pointB[i];
		manifold.points[i].normal = collisionInfo.normal[i];
		manifold.points[i].seperation = collisionInfo.seperation[i];

		// int64_t proxyIdA = m_fixtureA->getFixtureProxy()->proxyId;
		// int64_t proxyIdB = m_fixtureB->getFixtureProxy()->proxyId;

		// if (proxyIdA > proxyIdB)
		// {
		// 	int64_t tmp = proxyIdA;
		// 	proxyIdA = proxyIdB;
		// 	proxyIdB = tmp;
		// }

		// proxyIdA = (proxyIdA << 5) & bitmask;
		// proxyIdB = (proxyIdB << 5) & bitmask;
		// manifoldPoint.id = (proxyIdA << 32) | (proxyIdB << 10);
	}
}

void Contact::buildManifoldFromPolygon(CollisionInfo &collisionInfo, const Face &refFace, const Face &incFace,
									   std::vector<glm::vec3> &polygon, EpaInfo &epaInfo)
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
	if (denominator < 1e-8f)
	{
		ratio = 0.0f;
	}
	else
	{
		ratio = distance / denominator;
	}

	int32_t polygonCount = polygon.size();

	for (int32_t i = 0; i < polygonCount; ++i)
	{
		const glm::vec3 &point = polygon[i];

		// B측 point
		glm::vec3 pointB = point;

		// 침투깊이
		float penentration = ratio * (-(glm::dot(point, refN) - refPlaneDist));

		// A측 point
		glm::vec3 pointA = point + normal * penentration;

		// 접촉 정보
		collisionInfo.normal[i] = refN;
		collisionInfo.pointA[i] = pointA;
		collisionInfo.pointB[i] = pointB;
		collisionInfo.seperation[i] = penentration;
		++collisionInfo.size;
	}
}

std::vector<glm::vec3> Contact::clipPolygonAgainstPlane(const std::vector<glm::vec3> &polygon,
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

std::vector<glm::vec3> Contact::computeContactPolygon(const Face &refFace, const Face &incFace)
{
	// 초기 polygon: Incident Face의 4점
	std::vector<glm::vec3> poly = incFace.vertices;

	// Ref Face의 4개 엣지로 만들어지는 '4개 사이드 평면'에 대해 클리핑
	// refFace.vertices = v0,v1,v2,v3 라고 가정, CCW 형태
	const std::vector<glm::vec3> &vertices = refFace.vertices;
	int32_t len = vertices.size();
	for (int32_t i = 0; i < len; i++)
	{
		glm::vec3 start = vertices[i];
		glm::vec3 end = vertices[(i + 1) % len];

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

void Contact::sortPointsClockwise(std::vector<glm::vec3> &points, const glm::vec3 &center, const glm::vec3 &normal)
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

Face Contact::getBoxFace(const ConvexInfo &box, const glm::vec3 &normal)
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

	int32_t pointCount = box.pointsCount;
	for (int32_t i = 0; i < pointCount; ++i)
	{
		glm::vec3 &point = box.points[i];
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

Face Contact::getCylinderFace(const ConvexInfo &cylinder, const glm::vec3 &normal)
{
	Face face;

	glm::vec3 center;
	int32_t segments = 20;
	float length = glm::dot(normal, cylinder.axes[0]);
	float angleStep = 2.0f * glm::pi<float>() / static_cast<float>(segments);
	// std::cout << "normal: " << normal.x << " " << normal.y << " " << normal.z << "\n";
	if (length > 0.9f)
	{
		// std::cout << "top!!!\n";
		face.vertices.resize(segments);
		int32_t len = segments;
		for (int32_t i = 0; i < len; ++i)
		{
			face.vertices[i] = cylinder.points[i];
		}

		center = cylinder.center + cylinder.axes[0] * cylinder.height * 0.5f;
		face.normal = cylinder.axes[0];
		face.distance = glm::dot(cylinder.axes[0], face.vertices[0]);
	}
	else if (length < -0.9f)
	{
		// std::cout << "bottom!!!\n";

		int32_t len = segments * 2;
		face.vertices.resize(segments);
		for (int32_t i = segments; i < len; ++i)
		{
			face.vertices[i - segments] = cylinder.points[i];
		}

		center = cylinder.center - cylinder.axes[0] * cylinder.height * 0.5f;
		face.normal = -cylinder.axes[0];
		face.distance = glm::dot(-cylinder.axes[0], face.vertices[0]);
		// std::cout << "distance: " << face.distance << "\n";
	}
	else
	{
		// std::cout << "edge!!!\n";
		face.normal = normal;
		float dotResult = glm::dot(normal, cylinder.axes[0]);
		if (dotResult != 0.0f)
		{
			face.normal = glm::normalize(normal - dotResult * cylinder.axes[0]);
		}

		int32_t dir;
		float max = -FLT_MAX;
		// std::cout << "face.normal: (" << face.normal.x << ", " << face.normal.y << ", " << face.normal.z <<")\n";

		for (int32_t i = 1; i <= segments; ++i)
		{
			dotResult = glm::dot(cylinder.axes[i], face.normal);
			if (dotResult > max)
			{
				// std::cout << "max!!\n";
				// std::cout << "dotResult: " << dotResult <<" \n";
				// std::cout << "axes[" << i << "]: (" << cylinder.axes[i].x << ", " << cylinder.axes[i].y << ", " <<
				// cylinder.axes[i].z <<")\n";
				dir = i;
				max = dotResult;
			}
		}

		int32_t idx1 = dir - 1;
		int32_t idx2 = dir % segments;

		face.vertices.resize(4);
		face.vertices[0] = cylinder.points[idx1];
		face.vertices[1] = cylinder.points[idx2];
		face.vertices[2] = cylinder.points[idx1 + segments];
		face.vertices[3] = cylinder.points[idx2 + segments];

		// std::cout << "face 4 start\n";
		// for (int i = 0; i < 4; i ++)
		// {
		// std::cout << "face[" << i << "]: (" << face.vertices[i].x << ", " << face.vertices[i].y << ", " <<
		// face.vertices[i].z <<")\n";
		// }

		face.distance = glm::dot(face.normal, face.vertices[0]);
		center = (face.vertices[0] + face.vertices[1] + face.vertices[2] + face.vertices[3]) / 4.0f;
	}

	sortPointsClockwise(face.vertices, center, face.normal);

	return face;
}

Face Contact::getCapsuleFace(const ConvexInfo &capsule, const glm::vec3 &normal)
{
	Face face;

	int32_t segments = 20;
	float angleStep = 2.0f * glm::pi<float>() / static_cast<float>(segments);

	face.normal = normal;
	float dotResult = glm::dot(normal, capsule.axes[0]);

	if (dotResult != 0.0f)
	{
		face.normal = glm::normalize(normal - dotResult * capsule.axes[0]);
	}

	int32_t dir;
	float max = -FLT_MAX;
	// std::cout << "face.normal: (" << face.normal.x << ", " << face.normal.y << ", " << face.normal.z <<")\n";

	for (int32_t i = 1; i <= segments; ++i)
	{
		dotResult = glm::dot(capsule.axes[i], face.normal);
		if (dotResult > max)
		{
			// std::cout << "max!!\n";
			// std::cout << "dotResult: " << dotResult <<" \n";
			// std::cout << "axes[" << i << "]: (" << capsule.axes[i].x << ", " << capsule.axes[i].y << ", " <<
			// capsule.axes[i].z <<")\n";
			dir = i;
			max = dotResult;
		}
	}

	int32_t idx1 = dir - 1;
	int32_t idx2 = dir % segments;

	face.vertices.resize(4);
	face.vertices[0] = capsule.points[idx1];
	face.vertices[1] = capsule.points[idx2];
	face.vertices[2] = capsule.points[idx1 + segments];
	face.vertices[3] = capsule.points[idx2 + segments];

	// std::cout << "face 4 start\n";
	// for (int i = 0; i < 4; i ++)
	// {
	// std::cout << "face[" << i << "]: (" << face.vertices[i].x << ", " << face.vertices[i].y << ", " <<
	// face.vertices[i].z <<")\n";
	// }

	face.distance = glm::dot(face.normal, face.vertices[0]);
	glm::vec3 center = (face.vertices[0] + face.vertices[1] + face.vertices[2] + face.vertices[3]) / 4.0f;

	sortPointsClockwise(face.vertices, center, face.normal);

	return face;
}

bool Contact::isCollideToHemisphere(const ConvexInfo &capsule, const glm::vec3 &dir)
{
	return glm::length2(glm::dot(capsule.axes[0], dir)) > 1e-4f;
}

void Contact::addFaceInFaceArray(FaceArray &faceArray, int32_t idx1, int32_t idx2, int32_t idx3)
{
	int32_t count = faceArray.count;
	int32_t faceIdx = count * 3;

	if (count == faceArray.maxCount)
	{
		sizeUpFaceArray(faceArray, count * 2);
	}

	faceArray.faces[faceIdx] = idx1;
	faceArray.faces[faceIdx + 1] = idx2;
	faceArray.faces[faceIdx + 2] = idx3;
	++faceArray.count;
}

void Contact::mergeFaceArray(FaceArray &faceArray, FaceArray &newFaceArray)
{
	int32_t faceArrayCount = faceArray.count;
	int32_t newFaceArrayCount = newFaceArray.count;
	int32_t newCount = newFaceArrayCount + faceArrayCount;

	if (newCount > faceArray.maxCount)
	{
		int32_t newMaxCount = faceArrayCount;
		while (newMaxCount < newCount)
		{
			newMaxCount = newMaxCount * 2;
		}

		sizeUpFaceArray(faceArray, newMaxCount);
	}

	memcpy(faceArray.faces + faceArrayCount * 3, newFaceArray.faces, sizeof(int32_t) * newFaceArrayCount * 3);
	memcpy(faceArray.normals + faceArrayCount, newFaceArray.normals, sizeof(glm::vec4) * newFaceArrayCount);

	faceArray.count = newCount;
}

void Contact::sizeUpFaceArray(FaceArray &faceArray, int32_t newMaxCount)
{
	int32_t count = faceArray.count;
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(3 * newMaxCount * sizeof(int32_t));
	int32_t *newFaces = static_cast<int32_t *>(memory);
	memcpy(newFaces, faceArray.faces, sizeof(int32_t) * count * 3);

	memory = PhysicsAllocator::m_blockAllocator.allocateBlock(newMaxCount * sizeof(glm::vec4));
	glm::vec4 *newNormals = static_cast<glm::vec4 *>(memory);
	memcpy(newNormals, faceArray.normals, sizeof(glm::vec4) * count);

	PhysicsAllocator::m_blockAllocator.freeBlock(faceArray.faces, count * 3);
	PhysicsAllocator::m_blockAllocator.freeBlock(faceArray.normals, count);

	faceArray.maxCount = newMaxCount;
	faceArray.faces = newFaces;
	faceArray.normals = newNormals;
}

} // namespace ale