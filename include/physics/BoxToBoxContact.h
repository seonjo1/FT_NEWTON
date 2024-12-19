#ifndef BOXTOBOXCONTACT_H
#define BOXTOBOXCONTACT_H

#include "Contact.h"

namespace ale
{

struct BoxToBoxInfo
{
	bool collision;
	int32_t axisType;
	int32_t typeA;
	int32_t typeB;
	float overlap;
	glm::vec3 pointA;
	glm::vec3 pointB;
	glm::vec3 normal;
};

class BoxToBoxContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	BoxToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	virtual void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB) override;
	BoxToBoxInfo boxToBoxSAT(const std::vector<glm::vec3> &pointsA, const std::vector<glm::vec3> &pointsB,
							 const std::vector<glm::vec3> &axes);
	bool isOverlapped(BoxToBoxInfo &info, const glm::vec3 &axis, const std::vector<glm::vec3> &pointsA,
					  const std::vector<glm::vec3> &pointsB, int32_t axisType);
	inline void changeInfo(BoxToBoxInfo &info, float overlap, int32_t typeA, int32_t typeB, int32_t axisType);
	void fillFaceToPointInfo(BoxToBoxInfo &info, std::vector<glm::vec3> &pointsA, std::vector<glm::vec3> &pointsB);
	void fillEdgeToEdgeInfo(BoxToBoxInfo &info, std::vector<glm::vec3> &pointsA, std::vector<glm::vec3> &pointsB);
	void fillPointToFaceInfo(BoxToBoxInfo &info, std::vector<glm::vec3> &pointsA, std::vector<glm::vec3> &pointsB);
	bool isContainPoint(std::vector<int32_t> points, int32_t point);
	std::vector<glm::vec3> getProjectedEdge(const glm::vec3 &normal, const std::vector<glm::vec3> &edge);
	float findIntersectionRatio(const std::vector<glm::vec3> &edgeA, const std::vector<glm::vec3> &edgeB);
};
} // namespace ale

#endif