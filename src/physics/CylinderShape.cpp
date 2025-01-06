#include "physics/CylinderShape.h"
#include "physics/Contact.h"

namespace ale
{
CylinderShape::CylinderShape()
{
	m_type = Type::CYLINDER;
}

CylinderShape *CylinderShape::clone() const
{
	CylinderShape *clone = new CylinderShape();
	*clone = *this;
	return clone;
}

int32_t CylinderShape::getChildCount() const
{
	return 1;
}

void CylinderShape::computeAABB(AABB *aabb, const Transform &xf) const
{
	// get min, max vertex
	glm::vec3 upper = xf.position + glm::vec3(m_radius);
	glm::vec3 lower = xf.position - glm::vec3(m_radius);

	// std::cout << "upper: " << upper.x << ", " << upper.y << ", " << upper.z << '\n';
	// std::cout << "lower: " << lower.x << ", " << lower.y << ", " << lower.z << '\n';

	aabb->upperBound = upper + glm::vec3(0.1f);
	aabb->lowerBound = lower - glm::vec3(0.1f);
}
void CylinderShape::computeMass(MassData *massData, float density) const
{
}

void CylinderShape::findAxisByLongestPair(const std::vector<Vertex> &vertices)
{
	if (vertices.size() < 2)
	{
		m_axis = glm::vec3(0.0f);
		return;
	}

	float maxDistSq = -std::numeric_limits<float>::infinity();
	glm::vec3 bestA(0.0f), bestB(0.0f);

	// 1) 모든 점 쌍 탐색 -> 최대 거리 찾기
	for (size_t i = 0; i < vertices.size(); i++)
	{
		for (size_t j = i + 1; j < vertices.size(); j++)
		{
			float d2 = glm::length2(vertices[i].position - vertices[j].position);
			if (d2 > maxDistSq)
			{
				maxDistSq = d2;
				bestA = vertices[i].position;
				bestB = vertices[j].position;
			}
		}
	}

	// 2) 축 벡터
	glm::vec3 axis = glm::normalize(bestB - bestA);

	// 혹시나 0 벡터가 될 경우 대
	if (glm::length2(axis) < 1e-9f)
	{
		m_axis = glm::vec3(1.0f, 0.0f, 0.0f);
		m_height = 0.1f;
		return;
	}

	m_height = std::abs(glm::dot(bestA, axis) - glm::dot(bestB, axis));
	m_axis = axis;
}

void CylinderShape::computeCylinderCenter(const std::vector<Vertex> &vertices)
{
	glm::vec3 sum(0.0f);

	for (const Vertex &vertex : vertices)
	{
		sum += vertex.position;
	}

	localCenter = sum / static_cast<float>(vertices.size());
}

void CylinderShape::computeCylinderRadius(const std::vector<Vertex> &vertices)
{
	float maxRadius = 0.0f;
	for (const Vertex &vertex : vertices)
	{
		glm::vec3 diff = vertex.position - localCenter;
		float proj = glm::dot(diff, m_axis); // 축 방향 투영
		glm::vec3 onAxis = proj * m_axis;	   // 축 위 좌표
		glm::vec3 perp = diff - onAxis;	   // 축에 수직인 성분
		float dist = glm::length(perp);
		if (dist > maxRadius)
		{
			maxRadius = dist;
		}
	}
	m_radius = maxRadius;
}

void CylinderShape::setShapeFeatures(std::vector<Vertex> &vertices)
{
	computeCylinderCenter(vertices);
	findAxisByLongestPair(vertices);
	computeCylinderRadius(vertices);
}

float CylinderShape::getLocalRadius() const
{
	return 0.0f;
}

const glm::vec3 &CylinderShape::getLocalHalfSize() const
{
	return glm::vec3(0.0f);
}

ConvexInfo CylinderShape::getShapeInfo(const Transform &transform) const
{
	ConvexInfo cylinder;
	cylinder.radius = m_radius;
	cylinder.center = transform.toMatrix() * glm::vec4(localCenter, 1.0f);
	cylinder.axes[0] = transform.toMatrix() * glm::vec4(m_axis, 0.0f);
	cylinder.height = m_height;
	return cylinder;
}

} // namespace ale