#include "physics/CylinderShape.h"
#include "physics/Contact.h"

namespace ale
{
CylinderShape::CylinderShape()
{
	m_type = EType::CYLINDER;
}

CylinderShape *CylinderShape::clone() const
{
	void *memory = PhysicsAllocator::m_blockAllocator.allocateBlock(sizeof(CylinderShape));
	CylinderShape *clone = new (static_cast<CylinderShape *>(memory)) CylinderShape();
	*clone = *this;
	return clone;
}

int32_t CylinderShape::getChildCount() const
{
	return 1;
}

void CylinderShape::computeAABB(AABB *aabb, const Transform &xf) const
{
	// update vertices
	std::vector<glm::vec3> vertexVector(m_vertices.begin(), m_vertices.end());
	glm::mat4 transformMatrix = xf.toMatrix();

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

// void CylinderShape::findAxisByLongestPair(const std::vector<Vertex> &vertices)
// {
// 	if (vertices.size() < 2)
// 	{
// 		m_axis[0] = glm::vec3(0.0f);
// 		return;
// 	}

// 	float maxDistSq = -std::numeric_limits<float>::infinity();
// 	glm::vec3 bestA(0.0f), bestB(0.0f);

// 	// 1) 모든 점 쌍 탐색 -> 최대 거리 찾기
// 	for (size_t i = 0; i < vertices.size(); i++)
// 	{
// 		for (size_t j = i + 1; j < vertices.size(); j++)
// 		{
// 			float d2 = glm::length2(vertices[i].position - vertices[j].position);
// 			if (d2 > maxDistSq)
// 			{
// 				maxDistSq = d2;
// 				bestA = vertices[i].position;
// 				bestB = vertices[j].position;
// 			}
// 		}
// 	}

// 	// 2) 축 벡터
// 	glm::vec3 axis = glm::normalize(bestB - bestA);

// 	// 혹시나 0 벡터가 될 경우 대
// 	if (glm::length2(axis) < 1e-9f)
// 	{
// 		m_axis[0] = glm::vec3(1.0f, 0.0f, 0.0f);
// 		m_height = 0.1f;
// 		return;
// 	}

// 	m_height = std::abs(glm::dot(bestA, axis) - glm::dot(bestB, axis));
// 	m_axis[0] = axis;
// }

void CylinderShape::computeCylinderFeatures(const std::vector<Vertex> &vertices)
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

	m_center = (min + max) / 2.0f;
	m_axes[0] = glm::vec3(0.0f, 1.0f, 0.0f);
	m_height = max.y - min.y;

	m_radius = 0.0f;
	glm::vec2 center(m_center.x, m_center.z);
	for (const Vertex &vertex : vertices)
	{
		m_radius = std::max(m_radius, glm::length2(glm::vec2(vertex.position.x, vertex.position.z) - center));
	}
	m_radius = std::sqrt(m_radius);
}

// void CylinderShape::computeCylinderRadius(const std::vector<Vertex> &vertices)
// {
// 	float maxRadius = 0.0f;
// 	for (const Vertex &vertex : vertices)
// 	{
// 		glm::vec3 diff = vertex.position - localCenter;
// 		float proj = glm::dot(diff, m_axis[0]); // 축 방향 투영
// 		glm::vec3 onAxis = proj * m_axis[0];	   // 축 위 좌표
// 		glm::vec3 perp = diff - onAxis;	   // 축에 수직인 성분
// 		float dist = glm::length(perp);
// 		if (dist > maxRadius)
// 		{
// 			maxRadius = dist;
// 		}
// 	}

// 	const Vertex &vertex = vertices[0];
// 	glm::vec3 toVertex = vertex.position - localCenter;
// 	float dotResult = glm::dot(m_axis[0], toVertex);
// 	if (glm::length2(dotResult) == 0.0f)
// 	{
// 		m_axis[1] = glm::normalize(toVertex);
// 	}
// 	else
// 	{
// 		m_axis[1] = glm::normalize(toVertex - dotResult * m_axis[0]);
// 	}

// 	m_radius = maxRadius;
// }

void CylinderShape::createCylinderPoints()
{
	int32_t segments = 20;
	float angleStep = 2.0f * glm::pi<float>() / static_cast<float>(segments);
	glm::vec3 xAxis(1.0f, 0.0f, 0.0f);

	glm::vec4 topPoint = glm::vec4(m_center + m_height * 0.5f * m_axes[0] + xAxis * m_radius, 1.0f);
	glm::vec4 bottomPoint = glm::vec4(m_center - m_height * 0.5f * m_axes[0] + xAxis * m_radius, 1.0f);

	glm::quat quat = glm::angleAxis(angleStep / 2.0f, m_axes[0]);
	glm::mat4 mat = glm::toMat4(glm::normalize(quat));
	glm::vec3 dir = mat * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

	int32_t base = 0;
	for (int32_t i = base; i < segments; i++)
	{
		float theta = i * angleStep;
		glm::quat orientation = glm::angleAxis(theta, m_axes[0]);
		glm::mat4 rotationMatrix = glm::toMat4(glm::normalize(orientation));
		m_points[i] = rotationMatrix * topPoint;
		m_axes[i + 1] = rotationMatrix * glm::vec4(dir, 1.0f);
	}

	base = segments;
	for (int32_t i = base; i < segments + base; i++)
	{
		float theta = (i - base) * angleStep;
		glm::quat orientation = glm::angleAxis(theta, m_axes[0]);
		glm::mat4 rotationMatrix = glm::toMat4(glm::normalize(orientation));
		m_points[i] = rotationMatrix * bottomPoint;
	}

	// std::cout << "axes !!\n";
	// for (int i = 0; i <= segments; i++)
	// {
	// 	std::cout << "("<< m_axes[i].x << ", " << m_axes[i].y << ", " << m_axes[i].z << ")\n";
	// }

	// std::cout << "cylinder points!!\n";
	// for (int i = 0; i < segments * 2; i++)
	// {
	// 	std::cout << "("<< m_points[i].x << ", " << m_points[i].y << ", " << m_points[i].z << ")\n";
	// }
}

void CylinderShape::setShapeFeatures(const std::vector<Vertex> &vertices)
{
	computeCylinderFeatures(vertices);
	createCylinderPoints();
	// findAxisByLongestPair(vertices);
	// computeCylinderRadius(vertices);
}

ConvexInfo CylinderShape::getShapeInfo(const Transform &transform) const
{
	glm::mat4 matrix = transform.toMatrix();

	ConvexInfo cylinder;
	cylinder.radius = m_radius;
	cylinder.height = m_height;
	cylinder.center = matrix * glm::vec4(m_center, 1.0f);

	int32_t segments = 20;
	int32_t len = segments * 2;

	cylinder.axes.resize(segments + 1);
	cylinder.axes[0] = glm::normalize(matrix * glm::vec4(m_axes[0], 0.0f));

	for (int32_t i = 1; i <= segments; ++i)
	{
		cylinder.axes[i] = matrix * glm::vec4(m_axes[i], 1.0f);
	}

	cylinder.points.resize(len);
	// std::cout << "points size: " << cylinder.points.size() << " " << len << "\n";

	for (int32_t i = 0; i < segments; i++)
	{
		cylinder.points[i] = matrix * glm::vec4(m_points[i], 1.0f);
		cylinder.points[i + segments] = matrix * glm::vec4(m_points[i + segments], 1.0f);
	}

	// std::cout << "center: (" << cylinder.center.x << ", " << cylinder.center.y << ", " << cylinder.center.z << ")\n";

	// std::cout << "cylinder points!!\n";
	// for (int i = 0; i < cylinder.points.size(); i++)
	// {
	// 	std::cout << "("<< cylinder.points[i].x << ", " << cylinder.points[i].y << ", " << cylinder.points[i].z <<
	// ")\n";
	// }
	// std::cout << "bottom points)\n";
	// for (int i = 1; i < cylinder.points.size(); i = i + 2)
	// {
	// 	std::cout  << "(" << cylinder.points[i].x << ", " << cylinder.points[i].y << ", " << cylinder.points[i].z <<
	// ")\n";
	// }

	return cylinder;
}

} // namespace ale