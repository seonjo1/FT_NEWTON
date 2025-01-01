#ifndef BOXTOBOXCONTACT_H
#define BOXTOBOXCONTACT_H

#include "Contact.h"

namespace ale
{

struct CollisionPoints
{
	glm::vec3 normal;
	glm::vec3 pointA;
	glm::vec3 pointB;
	float seperation;
};

struct Simplex
{
	std::vector<glm::vec3> points;
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
	std::vector<CollisionPoints> getEpaResult(const BoxInfo &boxA, const BoxInfo &boxB, const Simplex &simplex);
	glm::vec3 supportBox(const BoxInfo &box, glm::vec3 dir);
	std::vector<glm::vec4> getCandidates(const BoxInfo &box, const glm::vec3 &dir);
	glm::vec3 getSupportPoint(const BoxInfo &boxA, const BoxInfo &boxB, glm::vec3 &dir);
	int32_t getFaceNormals(std::vector<glm::vec4> &normals, const std::vector<glm::vec3> &polytope,
						   const std::vector<int32_t> &faces);
	void addIfUniqueEdge(std::vector<std::pair<int32_t, int32_t>> &edges, const std::vector<int32_t> &faces, int32_t a,
						 int32_t b);
	bool handleLineSimplex(Simplex &simplex, glm::vec3 &dir);
	bool handleTriangleSimplex(Simplex &simplex, glm::vec3 &dir);
	bool handleTetrahedronSimplex(Simplex &simplex, glm::vec3 &dir);
	bool handleSimplex(Simplex &simplex, glm::vec3 &dir);
	bool getGjkResult(const BoxInfo &boxA, const BoxInfo &boxB, Simplex &simplex);
	bool isDuplicatedPoint(const std::vector<glm::vec3> &points, const glm::vec3 &supportPoint);
	bool isSameDirection(glm::vec3 v1, glm::vec3 v2);
	bool isSimilarDirection(glm::vec3 v1, glm::vec3 v2);
	bool isContained(const glm::vec3 &point, const BoxInfo &box, float seperation, float distance);

};
} // namespace ale

#endif