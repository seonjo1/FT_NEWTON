#ifndef BOXTOBOXCONTACT_H
#define BOXTOBOXCONTACT_H

#include "Contact.h"

namespace ale
{

class BoxToBoxContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	BoxToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	virtual glm::vec3 supportA(const ConvexInfo &box, glm::vec3 dir) override;
	virtual glm::vec3 supportB(const ConvexInfo &box, glm::vec3 dir) override;
	virtual void findCollisionPoints(const ConvexInfo &boxA, const ConvexInfo &boxB, std::vector<CollisionInfo> &collisionInfoVector,
							 EpaInfo &epaInfo, std::vector<Simplex> &simplexVector) override;
	
	Face getPolygonFace(const ConvexInfo &box, const glm::vec3 &normal);
	std::vector<glm::vec3> computeContactPolygon(const Face &refFace, const Face &incFace);
	std::vector<glm::vec3> clipPolygonAgainstPlane(const std::vector<glm::vec3> &polygon, const glm::vec3 &planeNormal,
												   float planeDist);

	void buildManifoldFromPolygon(std::vector<CollisionInfo> &collisionInfoVector, const Face &refFace,
								  const Face &incFace, std::vector<glm::vec3> &polygon, EpaInfo &epaInfo);
	void sortPointsClockwise(std::vector<glm::vec3> &points, const glm::vec3 &center, const glm::vec3 &normal);
};
} // namespace ale

#endif