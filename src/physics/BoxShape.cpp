#include "physics/BoxShape.h"
#include "physics/Contact.h"
#include <limits>

namespace ale
{
BoxShape::BoxShape()
{
	m_type = Type::BOX;
}
BoxShape *BoxShape::clone() const
{
	BoxShape *clone = new BoxShape();
	*clone = *this;
	return clone;
}
int32_t BoxShape::getChildCount() const
{
	return 1;
}
void BoxShape::computeAABB(AABB *aabb, const Transform &xf) const
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
void BoxShape::computeMass(MassData *massData, float density) const
{
}

void BoxShape::setVertices(const std::vector<Vertex> &vertices)
{
	glm::vec3 maxPos(std::numeric_limits<float>::lowest());
	glm::vec3 minPos(std::numeric_limits<float>::max());

	for (const Vertex &vertex : vertices)
	{
		maxPos.x = std::max(maxPos.x, vertex.position.x);
		maxPos.y = std::max(maxPos.y, vertex.position.y);
		maxPos.z = std::max(maxPos.z, vertex.position.z);
		minPos.x = std::min(minPos.x, vertex.position.x);
		minPos.y = std::min(minPos.y, vertex.position.y);
		minPos.z = std::min(minPos.z, vertex.position.z);

		m_vertices.insert(vertex.position);
	}

	localCenter = (maxPos + minPos) / 2.0f;
	halfSize = (maxPos - minPos) / 2.0f;
}

ConvexInfo BoxShape::getShapeInfo(const Transform &transform) const
{
	ConvexInfo box;
	glm::mat4 matrix = transform.toMatrix();

	box.center = matrix * glm::vec4(localCenter, 1.0f);
	box.halfSize = halfSize;
	box.points = {matrix * glm::vec4(localCenter - halfSize, 1.0f),
				  matrix * glm::vec4(localCenter + glm::vec3(halfSize.x, -halfSize.y, -halfSize.z), 1.0f),
				  matrix * glm::vec4(localCenter + glm::vec3(-halfSize.x, halfSize.y, -halfSize.z), 1.0f),
				  matrix * glm::vec4(localCenter + glm::vec3(-halfSize.x, -halfSize.y, halfSize.z), 1.0f),
				  matrix * glm::vec4(localCenter + glm::vec3(halfSize.x, halfSize.y, -halfSize.z), 1.0f),
				  matrix * glm::vec4(localCenter + glm::vec3(halfSize.x, -halfSize.y, halfSize.z), 1.0f),
				  matrix * glm::vec4(localCenter + glm::vec3(-halfSize.x, halfSize.y, halfSize.z), 1.0f),
				  matrix * glm::vec4(localCenter + halfSize, 1.0f)};
	glm::vec3 axisX = glm::normalize(box.points[1] - box.points[0]);
	glm::vec3 axisY = glm::normalize(box.points[2] - box.points[0]);
	glm::vec3 axisZ = glm::normalize(box.points[3] - box.points[0]);

	box.axes = {axisX, axisY, axisZ};

	return box;
}

float BoxShape::getLocalRadius() const
{
	return 0;
}

const glm::vec3 &BoxShape::getLocalHalfSize() const
{
	return halfSize;
}

} // namespace ale