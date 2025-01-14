#ifndef CAPSULESHAPE_H
#define CAPSULESHAPE_H

#include "physics/Shape.h"

struct ConvexInfo;

namespace ale
{
class CapsuleShape : public Shape
{
  public:
	CapsuleShape();
	virtual ~CapsuleShape() = default;
	CapsuleShape *clone() const;
	int32_t getChildCount() const;
	void computeAABB(AABB *aabb, const Transform &xf) const;
	void computeMass(MassData *massData, float density) const;
	void setShapeFeatures(const std::vector<Vertex> &vertices);
	void computeCapsuleFeatures(const std::vector<Vertex> &vertices);
	void createCapsulePoints();

	virtual float getLocalRadius() const override;
	virtual const glm::vec3 &getLocalHalfSize() const override;
	virtual ConvexInfo getShapeInfo(const Transform &transform) const override;

	float m_radius;
	float m_height;
	glm::vec3 m_axes[21];
	glm::vec3 m_points[40];
	std::set<glm::vec3, Vec3Comparator> m_vertices;
};
} // namespace ale
#endif