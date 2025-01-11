#include "physics/BoxToCapsuleContact.h"

namespace ale
{

BoxToCapsuleContact::BoxToCapsuleContact(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
	: Contact(fixtureA, fixtureB, indexA, indexB) {};

Contact *BoxToCapsuleContact::create(Fixture *fixtureA, Fixture *fixtureB, int32_t indexA, int32_t indexB)
{
	return new BoxToCapsuleContact(fixtureA, fixtureB, indexA, indexB);
}

glm::vec3 BoxToCapsuleContact::supportA(const ConvexInfo &box, glm::vec3 dir)
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

glm::vec3 BoxToCapsuleContact::supportB(const ConvexInfo &capsule, glm::vec3 dir)
{
	float dotResult = glm::dot(dir, capsule.axes[0]);
	if (dotResult > 0)
	{
		return capsule.points[0] + dir * capsule.radius;
	}
	else
	{
		return capsule.points[1] + dir * capsule.radius;
	}
}

void BoxToCapsuleContact::findCollisionPoints(const ConvexInfo &box, const ConvexInfo &capsule,
											  std::vector<CollisionInfo> &collisionInfoVector, EpaInfo &epaInfo,
											  std::vector<Simplex> &simplexVector)
{
	if (isCollideToHemisphere(capsule, -epaInfo.normal))
	{
		CollisionInfo collisionInfo;

		collisionInfo.normal = epaInfo.normal;
		collisionInfo.seperation = epaInfo.distance;
		// std::cout << "epaInfo.normal: " << epaInfo.normal.x << " " << epaInfo.normal.y << " " << epaInfo.normal.z << "\n";
		if (glm::dot(capsule.axes[0], collisionInfo.normal) < 0)
		{
			// std::cout << "upper hemisphere\n";
			collisionInfo.pointB = capsule.points[0] + collisionInfo.normal * capsule.radius;
			collisionInfo.pointA = collisionInfo.pointB - collisionInfo.normal * collisionInfo.seperation;
		}
		else
		{
			// std::cout << "lower hemisphere\n";
			collisionInfo.pointB = capsule.points[1] + collisionInfo.normal * capsule.radius;
			collisionInfo.pointA = collisionInfo.pointB - collisionInfo.normal * collisionInfo.seperation;
			// std::cout << "pointB: " << collisionInfo.pointB.x << " " << collisionInfo.pointB.y << " " << collisionInfo.pointB.z << "\n";
			// std::cout << "pointA: " << collisionInfo.pointA.x << " " << collisionInfo.pointA.y << " " << collisionInfo.pointA.z << "\n";
		}
		
		collisionInfoVector.push_back(collisionInfo);
	}
	else
	{
		// std::cout << "edge!!!\n";
		Face refFace = getBoxFace(box, epaInfo.normal);
		Face incFace = getCapsuleFace(capsule, -epaInfo.normal);

		std::vector<glm::vec3> contactPolygon = computeContactPolygon(refFace, incFace);
		buildManifoldFromPolygon(collisionInfoVector, refFace, incFace, contactPolygon, epaInfo);
	}
}

} // namespace ale