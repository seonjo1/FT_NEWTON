#include "physics/BoxShape.h"

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

	for (glm::vec3 &vertex : vertexVector)
	{
		glm::vec4 v = transformMatrix * glm::vec4(vertex, 1.0f);
		vertex = glm::vec3(v.x, v.y, v.z);
	}

	std::sort(vertexVector.begin(), vertexVector.end(), Vec3Comparator());

	// get min, max vertex - 가장 앞과 끝 값만 가져오면 되는 게 아니라 가장 큰 x,y,z 작은 x,y,z 필요
	glm::vec3 upper = *std::prev(vertexVector.end());
	glm::vec3 lower = *vertexVector.begin();

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
	for (const Vertex &vertex : vertices)
	{
		m_vertices.insert(vertex.position);
	}
}

} // namespace ale