#ifndef CONTACT_H
#define CONTACT_H

#include "Fixture.h"
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
	glm::vec3 normal;
	glm::vec3 pointA;
	glm::vec3 pointB;
	float seperation;
};

struct Simplex
{
	glm::vec3 diff;
	glm::vec3 a;
	glm::vec3 b;
};

struct ConvexInfo
{
	std::vector<glm::vec3> points;
	std::vector<glm::vec3> axes;
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

	float getFriction() const;
	float getRestitution() const;
	float getTangentSpeed() const;
	Contact *getNext();
	Fixture *getFixtureA() const;
	Fixture *getFixtureB() const;
	int32_t getChildIndexA() const;
	int32_t getChildIndexB() const;
	ContactLink *getNodeA();
	ContactLink *getNodeB();
	const Manifold &getManifold() const;

	void setPrev(Contact *contact);
	void setNext(Contact *contact);
	void setFlag(EContactFlag flag);
	void unsetFlag(EContactFlag flag);
	bool hasFlag(EContactFlag flag);

  protected:
	static contactMemberFunction createContactFunctions[16];

	Simplex getSupportPoint(const ConvexInfo &convexA, const ConvexInfo &convexB, glm::vec3 &dir);
	bool handleLineSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool handleTriangleSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool handleTetrahedronSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool handleSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool getGjkResult(const ConvexInfo &convexA, const ConvexInfo &convexB, std::vector<Simplex> &simplex);
	bool isDuplicatedPoint(const std::vector<Simplex> &simplexVector, const glm::vec3 &supportPoint);
	bool isSameDirection(glm::vec3 v1, glm::vec3 v2);
	bool isSimilarDirection(glm::vec3 v1, glm::vec3 v2);
	EpaInfo getEpaResult(const ConvexInfo &convexA, const ConvexInfo &convexB, std::vector<Simplex> &simplex);
	int32_t getFaceNormals(std::vector<glm::vec4> &normals, const std::vector<Simplex> &simplexVector,
						   std::vector<int32_t> &faces);
	void addIfUniqueEdge(std::vector<std::pair<int32_t, int32_t>> &edges, const std::vector<int32_t> &faces, int32_t a,
						 int32_t b);
	void generateManifolds(std::vector<CollisionInfo> &collisionInfoVector, Manifold &manifold, Fixture *m_fixtureA, Fixture *m_fixtureB);	

	virtual glm::vec3 supportA(const ConvexInfo &box, glm::vec3 dir) = 0;
	virtual glm::vec3 supportB(const ConvexInfo &box, glm::vec3 dir) = 0;
	virtual void findCollisionPoints(const ConvexInfo &boxA, const ConvexInfo &boxB, std::vector<CollisionInfo> &collisionInfoVector,
							 EpaInfo &epaInfo, std::vector<Simplex> &simplexVector) = 0;

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

	float m_friction;
	float m_restitution;

	float m_tangentSpeed;
};
} // namespace ale

#endif