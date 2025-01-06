#ifndef CYLINDERSHAPE_H
#define CYLINDERSHAPE_H

#include "physics/Shape.h"

struct ConvexInfo;

namespace ale
{
class CylinderShape : public Shape
{
  public:
	CylinderShape();
	virtual ~CylinderShape() = default;
	CylinderShape *clone() const;
	int32_t getChildCount() const;
	void computeAABB(AABB *aabb, const Transform &xf) const;
	void computeMass(MassData *massData, float density) const;
	void setShapeFeatures(std::vector<Vertex> &vertices);
	void findAxisByLongestPair(const std::vector<Vertex> &vertices);
	void computeCylinderRadius(const std::vector<Vertex> &vertices);
	void computeCylinderCenter(const std::vector<Vertex> &vertices);

	virtual float getLocalRadius() const override;
	virtual const glm::vec3 &getLocalHalfSize() const override;
	virtual ConvexInfo getShapeInfo(const Transform &transform) const override;

	float m_radius;
	float m_height;
	glm::vec3 m_axis;
};
} // namespace ale
#endif