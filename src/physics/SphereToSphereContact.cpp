#include "physics/SphereToSphereContact.h"

namespace ale
{

SphereToSphereContact::SphereToSphereContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *SphereToSphereContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new SphereToSphereContact(fixtureA, fixtureB, indexA, indexB);
}

void SphereToSphereContact::evaluate(Manifold &manifold, const Transform &transformA, const Transform &transformB)
{
	/*
		64bit = 27bit(small proxyId) 	 |
				27bit(big proxyId) 		 |
				5bit(small contact part) |
				5bit(big contact part)
	*/
	static int64_t bitmask = 0xFFFFFFFF & ~0b11111;

	ManifoldPoint manifoldPoint;
	manifoldPoint.type = EManifoldType::FACE_A_TO_FACE_B;

	Shape *shapeA = m_fixtureA->getShape();
	Shape *shapeB = m_fixtureB->getShape();

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

	glm::vec3 worldCenterA = transformA.toMatrix() * glm::vec4(shapeA->localCenter, 1.0f);
	glm::vec3 worldCenterB = transformB.toMatrix() * glm::vec4(shapeB->localCenter, 1.0f);

	manifoldPoint.normal = glm::normalize(worldCenterB - worldCenterA);
	manifoldPoint.pointA = worldCenterB - shapeB->getLocalRadius() * manifoldPoint.normal;
	manifoldPoint.pointB = worldCenterA + shapeA->getLocalRadius() * manifoldPoint.normal;
	manifoldPoint.seperation = glm::length(manifoldPoint.pointA - manifoldPoint.pointB);

	manifold.points.push_back(manifoldPoint);
}

} // namespace ale