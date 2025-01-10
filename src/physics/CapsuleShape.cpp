#include "physics/CapsuleShape.h"
#include "physics/Contact.h"

namespace ale
{
CapsuleShape::CapsuleShape()
{
	m_type = Type::CAPSULE;
}

CapsuleShape *CapsuleShape::clone() const
{
	CapsuleShape *clone = new CapsuleShape();
	*clone = *this;
	return clone;
}

int32_t CapsuleShape::getChildCount() const
{
	return 1;
}

void CapsuleShape::computeAABB(AABB *aabb, const Transform &xf) const
{
	// update vertices
	std::vector<glm::vec3> vertexVector(m_vertices.begin(), m_vertices.end());
	glm::mat4 rotationMatrix = glm::toMat4(glm::normalize(xf.orientation));
	glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), xf.position);
	glm::mat4 transformMatrix = translationMatrix * rotationMatrix;

	glm::vec3 upper(std::numeric_limits<float>::lowest());
	glm::vec3 lower(std::numeric_limits<float>::max());

	for (glm::vec3 &vertex : vertexVector)
	{
		glm::vec4 v = transformMatrix * glm::vec4(vertex, 1.0f);
		vertex = glm::vec3(v.x, v.y, v.z);

		upper.x = std::max(upper.x, vertex.x);
		upper.y = std::max(upper.y, vertex.y);
		upper.z = std::max(upper.z, vertex.z);
		lower.x = std::min(lower.x, vertex.x);
		lower.y = std::min(lower.y, vertex.y);
		lower.z = std::min(lower.z, vertex.z);
	}

	// 최적화 여지 있음.
	// std::sort(vertexVector.begin(), vertexVector.end(), Vec3Comparator());

	// glm::vec3 upper = *std::prev(vertexVector.end());
	// glm::vec3 lower = *vertexVector.begin();

	// std::cout << "upper: " << upper.x << ", " << upper.y << ", " << upper.z << '\n';
	// std::cout << "lower: " << lower.x << ", " << lower.y << ", " << lower.z << '\n';

	aabb->upperBound = upper + glm::vec3(0.1f);
	aabb->lowerBound = lower - glm::vec3(0.1f);
}

void CapsuleShape::computeMass(MassData *massData, float density) const
{
}

void CapsuleShape::setShapeFeatures(const std::vector<Vertex> &vertices)
{
	glm::vec3 min(FLT_MAX);
	glm::vec3 max(-FLT_MAX);
	
	for (const Vertex &vertex : vertices)
	{
		max.x = std::max(vertex.position.x, max.x);
		max.y = std::max(vertex.position.y, max.y);
		max.z = std::max(vertex.position.z, max.z);

		min.x = std::min(vertex.position.x, min.x);
		min.y = std::min(vertex.position.y, min.y);
		min.z = std::min(vertex.position.z, min.z);

		m_vertices.insert(vertex.position);
	}

	localCenter = (min + max) / 2.0f;
	m_axis[0] = glm::vec3(0.0f, 1.0f, 0.0f);
	m_axis[1] = glm::vec3(1.0f, 0.0f, 0.0f);
	m_height = max.y - min.y;

	m_radius = 0.0f;
	glm::vec2 center(localCenter.x, localCenter.z);
	for (const Vertex &vertex : vertices)
	{
		m_radius = std::max(m_radius, glm::length2(glm::vec2(vertex.position.x, vertex.position.z) - center));
	}
	m_radius = std::sqrt(m_radius);
}

float CapsuleShape::getLocalRadius() const
{
	return 0.0f;
}

const glm::vec3 &CapsuleShape::getLocalHalfSize() const
{
	return glm::vec3(0.0f);
}

ConvexInfo CapsuleShape::getShapeInfo(const Transform &transform) const
{
	ConvexInfo cylinder;
	cylinder.radius = m_radius;
	cylinder.center = transform.toMatrix() * glm::vec4(localCenter, 1.0f);
	cylinder.axes.push_back(transform.toMatrix() * glm::vec4(m_axis[0], 0.0f));

	cylinder.height = m_height;
	glm::vec3 topPoint = transform.toMatrix() * glm::vec4(localCenter + m_height * 0.5f * m_axis[0] + m_axis[1] * m_radius, 1.0f);
	glm::vec3 bottomPoint = transform.toMatrix() * glm::vec4(localCenter - m_height * 0.5f * m_axis[0] + m_axis[1] * m_radius, 1.0f);
	cylinder.points.push_back(topPoint);
	cylinder.points.push_back(bottomPoint);
	// std::cout << "topPoint: " << topPoint.x << " " << topPoint.y << " " << topPoint.z << "\n";
	// std::cout << "bottomPoint: " << bottomPoint.x << " " << bottomPoint.y << " " << bottomPoint.z << "\n";
	return cylinder;
}

} // namespace ale