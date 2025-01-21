#ifndef CONTACT_H
#define CONTACT_H

#include "Fixture.h"
#include "PhysicsAllocator.h"
#include <cmath>

namespace ale
{

struct Face
{
	glm::vec3 normal;
	float distance;
	std::vector<glm::vec3> vertices;
};

struct CollisionInfo
{
	glm::vec3 normal[MAX_MANIFOLD_COUNT];
	glm::vec3 pointA[MAX_MANIFOLD_COUNT];
	glm::vec3 pointB[MAX_MANIFOLD_COUNT];
	float seperation[MAX_MANIFOLD_COUNT];
	int32_t size;
};

const int32_t MAX_SIMPLEX_COUNT = 256;

struct FaceArray
{
	int32_t count;
	int32_t maxCount;
	int32_t *faces;
	glm::vec4 *normals;

	FaceArray()
	{
		count = 0;
		maxCount = 64;

		void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(3 * maxCount * sizeof(int32_t));
		faces = static_cast<int32_t *>(memory);

		memory = PhysicsAllocator::m_blockAllocator.allocateBlock(maxCount * sizeof(glm::vec4));
		normals = static_cast<glm::vec4 *>(memory);
	}
};

struct Simplex
{
	glm::vec3 diff;
	glm::vec3 a;
	glm::vec3 b;
};

struct SimplexArray
{
	Simplex simplices[MAX_SIMPLEX_COUNT];
	int32_t simplexCount;
};

struct ConvexInfo
{
	glm::vec3 *points;
	glm::vec3 *axes;
	int32_t pointsCount;
	int32_t axesCount;
	glm::vec3 halfSize;
	glm::vec3 center;
	float radius;
	float height;
};

struct EpaInfo
{
	glm::vec3 normal;
	float distance;
};

class Contact;
struct Manifold;

using contactMemberFunction = Contact *(*)(Fixture *, Fixture *, int32_t, int32_t);

struct ContactLink
{
	Rigidbody *other;  // 연결된 반대쪽 Body
	Contact *contact;  // 두 Body 간의 Contact 정보
	ContactLink *prev; // 이전 충돌 정보
	ContactLink *next; // 다음 충돌 정보

	ContactLink() : other(nullptr), contact(nullptr), prev(nullptr), next(nullptr) {};
};

enum class EContactFlag
{
	ISLAND = (1 << 0),
	TOUCHING = (1 << 2),
};

int32_t operator&(int32_t val, EContactFlag flag);
int32_t operator|(int32_t val, EContactFlag flag);
int32_t operator~(EContactFlag flag);
bool operator==(int32_t val, EContactFlag flag);

class Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	Contact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	void update();
	void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB);

	void generateManifolds(CollisionInfo &collisionInfo, Manifold &manifold, Fixture *m_fixtureA, Fixture *m_fixtureB);
	float getFriction() const;
	float getRestitution() const;
	int32_t getChildIndexA() const;
	int32_t getChildIndexB() const;
	int32_t getFaceNormals(SimplexArray &simplexArray, FaceArray &faceArray);
	Contact *getNext();
	Simplex getSupportPoint(const ConvexInfo &convexA, const ConvexInfo &convexB, glm::vec3 &dir);
	EpaInfo getEpaResult(const ConvexInfo &convexA, const ConvexInfo &convexB, SimplexArray &simplexArray);
	Fixture *getFixtureA() const;
	Fixture *getFixtureB() const;
	ContactLink *getNodeA();
	ContactLink *getNodeB();
	Manifold &getManifold();

	void setPrev(Contact *contact);
	void setNext(Contact *contact);
	void setFlag(EContactFlag flag);
	bool hasFlag(EContactFlag flag);
	void unsetFlag(EContactFlag flag);

  protected:
	static contactMemberFunction createContactFunctions[32];

	bool handleLineSimplex(SimplexArray &simplexArray, glm::vec3 &dir);
	bool handleTriangleSimplex(SimplexArray &simplexArray, glm::vec3 &dir);
	bool handleTetrahedronSimplex(SimplexArray &simplexArray, glm::vec3 &dir);
	bool handleSimplex(SimplexArray &simplexArray, glm::vec3 &dir);
	bool getGjkResult(const ConvexInfo &convexA, const ConvexInfo &convexB, SimplexArray &simplexArray);
	bool checkSphereToSphereCollide(const ConvexInfo &convexA, const ConvexInfo &convexB);
	bool isDuplicatedPoint(const SimplexArray &simplexArray, const glm::vec3 &supportPoint);
	bool isSameDirection(glm::vec3 v1, glm::vec3 v2);
	bool isSimilarDirection(glm::vec3 v1, glm::vec3 v2);
	void addIfUniqueEdge(std::vector<std::pair<int32_t, int32_t>> &edges, const int32_t *faces, int32_t a, int32_t b);

	virtual glm::vec3 supportA(const ConvexInfo &convexA, glm::vec3 dir) = 0;
	virtual glm::vec3 supportB(const ConvexInfo &convexB, glm::vec3 dir) = 0;
	virtual void findCollisionPoints(const ConvexInfo &convexA, const ConvexInfo &convexB, CollisionInfo &collisionInfo,
									 EpaInfo &epaInfo, SimplexArray &simplexArray) = 0;

	std::vector<glm::vec3> computeContactPolygon(const Face &refFace, const Face &incFace);
	std::vector<glm::vec3> clipPolygonAgainstPlane(const std::vector<glm::vec3> &polygon, const glm::vec3 &planeNormal,
												   float planeDist);

	void buildManifoldFromPolygon(CollisionInfo &collisionInfo, const Face &refFace, const Face &incFace,
								  std::vector<glm::vec3> &polygon, EpaInfo &epaInfo);
	void sortPointsClockwise(std::vector<glm::vec3> &points, const glm::vec3 &center, const glm::vec3 &normal);

	Face getBoxFace(const ConvexInfo &box, const glm::vec3 &normal);
	Face getCylinderFace(const ConvexInfo &cylinder, const glm::vec3 &normal);
	Face getCapsuleFace(const ConvexInfo &capsule, const glm::vec3 &normal);

	bool isCollideToHemisphere(const ConvexInfo &capsule, const glm::vec3 &dir);

	void addFaceInFaceArray(FaceArray &faceArray, int32_t idx1, int32_t idx2, int32_t idx3);
	void mergeFaceArray(FaceArray &faceArray, FaceArray &newFaceArray);
	void sizeUpFaceArray(FaceArray &faceArray, int32_t newMaxCount);

	float m_friction;
	float m_restitution;
	int32_t m_flags;
	Contact *m_prev;
	Contact *m_next;
	ContactLink m_nodeA;
	ContactLink m_nodeB;
	Fixture *m_fixtureA;
	Fixture *m_fixtureB;
	int32_t m_indexA;
	int32_t m_indexB;
	Manifold m_manifold;
};
} // namespace ale

#endif