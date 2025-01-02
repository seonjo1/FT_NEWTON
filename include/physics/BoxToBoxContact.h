#ifndef BOXTOBOXCONTACT_H
#define BOXTOBOXCONTACT_H

#include "Contact.h"

namespace ale
{
struct Face
{
	glm::vec3 normal;				 // 면 법선(월드 좌표)
	float distance;					 // 면 방정식: dot(normal, X) - distance = 0
	std::vector<glm::vec3> vertices; // 면의 꼭지점(4개)
};

struct CollisionPoints
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

struct BoxInfo
{
	std::vector<glm::vec3> points;
	std::vector<glm::vec3> axes;
	glm::vec3 halfSize;
	glm::vec3 center;
};

class BoxToBoxContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	BoxToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	virtual void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB) override;
	std::vector<CollisionPoints> getEpaResult(const BoxInfo &boxA, const BoxInfo &boxB, std::vector<Simplex> &simplex);
	glm::vec3 supportBox(const BoxInfo &box, glm::vec3 dir);
	std::vector<glm::vec4> getCandidates(const BoxInfo &box, const glm::vec3 &dir);
	Simplex getSupportPoint(const BoxInfo &boxA, const BoxInfo &boxB, glm::vec3 &dir);
	int32_t getFaceNormals(std::vector<glm::vec4> &normals, const std::vector<Simplex> &simplexVector,
						   const std::vector<int32_t> &faces);
	void addIfUniqueEdge(std::vector<std::pair<int32_t, int32_t>> &edges, const std::vector<int32_t> &faces, int32_t a,
						 int32_t b);
	bool handleLineSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool handleTriangleSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool handleTetrahedronSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool handleSimplex(std::vector<Simplex> &simplexVector, glm::vec3 &dir);
	bool getGjkResult(const BoxInfo &boxA, const BoxInfo &boxB, std::vector<Simplex> &simplex);
	bool isDuplicatedPoint(const std::vector<Simplex> &simplexVector, const glm::vec3 &supportPoint);
	bool isSameDirection(glm::vec3 v1, glm::vec3 v2);
	bool isSimilarDirection(glm::vec3 v1, glm::vec3 v2);
	Face getPolygonFace(const BoxInfo &box, const glm::vec3 &normal);
	std::vector<glm::vec3> computeContactPolygon(const Face &refFace, const Face &incFace);
	std::vector<glm::vec3> clipPolygonAgainstPlane(const std::vector<glm::vec3> &polygon, const glm::vec3 &planeNormal,
												   float planeDist);

	std::vector<CollisionPoints> buildManifoldFromPolygon(const Face &refFace, const Face &incFace,
														  std::vector<glm::vec3> &polygon, const glm::vec3 &normal,
														  float maxPenetration);
	void sortPointsClockwise(std::vector<glm::vec3> &points, const glm::vec3 &center, const glm::vec3 &normal);
};
} // namespace ale

#endif