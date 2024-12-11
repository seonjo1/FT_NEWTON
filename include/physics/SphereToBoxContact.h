#ifndef SPHERETOBOXCONTACT_H
#define SPHERETOBOXCONTACT_H

#include "Contact.h"

namespace ale
{

struct SphereToBoxInfo
{
	int32_t type;
	float distance;
	glm::vec3 point;
	glm::vec3 normal;
};

class SphereToBoxContact : public Contact
{
  public:
	static Contact *create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);

	SphereToBoxContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB);
	virtual void evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB) override;
	void getPointToLineDistance(const glm::vec3& center, const glm::vec3& p1, const glm::vec3& p2, int32_t type, SphereToBoxInfo& info);
	void getPointToFaceDistance(const glm::vec3& center, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& p4,
								 int32_t type, SphereToBoxInfo& info);
};
} // namespace ale

#endif