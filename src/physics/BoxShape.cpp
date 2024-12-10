#include "physics/BoxShape.h"

namespace ale
{
BoxShape::BoxShape()
{
	type = Type::e_box;
}
BoxShape *BoxShape::clone() const
{
	BoxShape *clone = new BoxShape();
	*clone = *this;
	return clone;
}
int32_t BoxShape::GetChildCount() const
{
	return 1;
}
void BoxShape::ComputeAABB(AABB *aabb, const Transform &xf) const
{
	// update vertices
	std::vector<glm::vec3> vertexVector(vertices.begin(), vertices.end());
	glm::mat4 rotationMatrix = glm::toMat4(glm::normalize(xf.orientation));
	glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), xf.position);
	glm::mat4 transformMatrix = translationMatrix * rotationMatrix;

	for (glm::vec3 &vertex : vertexVector)
	{
		glm::vec4 v = transformMatrix * glm::vec4(vertex, 1.0f);
		vertex = glm::vec3(v.x, v.y, v.z);
	}

	std::sort(vertexVector.begin(), vertexVector.end(), Vec3Comparator());

	// get min, max vertex
	glm::vec3 upper = *std::prev(vertexVector.end());
	glm::vec3 lower = *vertexVector.begin();

	// std::cout << "upper: " << upper.x << ", " << upper.y << ", " << upper.z << '\n';
	// std::cout << "lower: " << lower.x << ", " << lower.y << ", " << lower.z << '\n';

	aabb->upperBound = upper + glm::vec3(0.1f);
	aabb->lowerBound = lower - glm::vec3(0.1f);
}
void BoxShape::ComputeMass(MassData *massData, float density) const
{
}

void BoxShape::SetVertices(const std::vector<Vertex> &vertices)
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

		this->vertices.insert(vertex.position);
	}

	localCenter = (maxPos + minPos) / 2.0f;
	halfSize = (maxPos - minPos) / 2.0f;
}

} // namespace ale