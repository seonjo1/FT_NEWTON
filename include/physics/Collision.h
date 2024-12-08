#ifndef COLLISION_H
#define COLLISION_H

#include "common.h"

namespace ale
{
struct AABB
{
	bool IsValid() const;

	glm::vec3 GetCenter() const
	{
	}

	glm::vec3 GetExtents() const
	{
	}

	float GetSurface() const
	{
		glm::vec3 v = upperBound - lowerBound;

		return (v.x * v.y + v.y * v.z + v.z * v.x) * 2.0f;
	}

	void Combine(const AABB &aabb)
	{
		lowerBound = min(lowerBound, aabb.lowerBound);
		upperBound = max(upperBound, aabb.upperBound);
	}

	void Combine(const AABB &aabb1, const AABB &aabb2)
	{
		lowerBound = min(aabb1.lowerBound, aabb2.lowerBound);
		upperBound = max(aabb1.upperBound, aabb2.upperBound);
	}

	bool Contains(const AABB &aabb) const
	{
	}

	glm::vec3 lowerBound;
	glm::vec3 upperBound;
};

// translation, rotation
struct Transform
{
	Transform()
	{
	}
	Transform(const glm::vec3 &p, const glm::quat &r) : position(p), orientation(r)
	{
	}
	void Set(const glm::vec3 &p, float angle)
	{
		position = p;
		// set orientation by angle
	}

	glm::vec3 position;
	glm::quat orientation;
};

inline bool testOverlap(const AABB &a, const AABB &b)
{
	glm::vec3 d1, d2;

	d1 = b.lowerBound - a.upperBound;
	d2 = a.lowerBound - b.upperBound;

	if (d1.x > 0 || d1.y > 0 || d1.z > 0)
		return false;
	if (d2.x > 0 || d2.y > 0 || d2.z > 0)
		return false;
	return true;
}

enum class EManifoldType
{
	FACE_A_TO_POINT_B,
	POINT_A_TO_FACE_B,
	EDGE_A_TO_FACE_B,
	FACE_A_TO_EDGE_B,
	EDGE_A_TO_EDGE_B,
	FACE_A_TO_FACE_B,
};

// struct ContactFeature
// {
// 	enum Type
// 	{
// 		e_vertex = 0,
// 		e_face = 1
// 	};

// 	uint8_t indexA;		///< Feature index on shapeA
// 	uint8_t indexB;		///< Feature index on shapeB
// 	uint8_t typeA;		///< The feature type on shapeA
// 	uint8_t typeB;		///< The feature type on shapeB
// };

union ContactID
{
	// ContactFeature cf;
	uint32_t key;					///< Used to quickly compare contact ids.
};

struct ManifoldPoint
{
	glm::vec2 localPoint;		// 충돌 지점의 위치
	float normalImpulse;	// 법선 방향 충격량
	float tangentImpulse;	// 접촉면 충격량
	ContactID id;			// 충돌 지점의 고유 id
};

struct Manifold
{
	ManifoldPoint points[b2_maxManifoldPoints];	// 접촉점, 충격량, 충돌 지점의 고유 id
	glm::vec2 localNormal;						// Local 좌표계에서의 법선 벡터
	glm::vec2 localPoint;						// Local 좌표계에서 충돌 면을 특정하기 위한 좌표
	EManifoldType type;							// 타입
	int32_t pointCount;							// 접촉점 개수
};

} // namespace ale
#endif