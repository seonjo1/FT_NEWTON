#ifndef Vertex_H
#define Vertex_H

#include "Common.h"

struct Vertex
{
	glm::vec3 position;
	glm::vec3 normal;
	glm::vec2 texCoord;

	// 정점 데이터가 전달되는 방법을 알려주는 구조체 반환하는 함수
	static VkVertexInputBindingDescription getBindingDescription()
	{
		// 파이프라인에 정점 데이터가 전달되는 방법을 알려주는 구조체
		VkVertexInputBindingDescription bindingDescription{};
		bindingDescription.binding = 0; // 버텍스 바인딩 포인트 (현재 0번에 vertex 정보 바인딩)
		bindingDescription.stride = sizeof(Vertex); // 버텍스 1개 단위의 정보 크기
		bindingDescription.inputRate =
			VK_VERTEX_INPUT_RATE_VERTEX; // 정점 데이터 처리 방법
										 // 1. VK_VERTEX_INPUT_RATE_VERTEX : 정점별로 데이터 처리
										 // 2. VK_VERTEX_INPUT_RATE_INSTANCE : 인스턴스별로 데이터 처리
		return bindingDescription;
	}

	// 정점 속성별 데이터 형식과 위치를 지정하는 구조체 반환하는 함수
	static std::array<VkVertexInputAttributeDescription, 3> getAttributeDescriptions()
	{
		// 정점 속성의 데이터 형식과 위치를 지정하는 구조체
		std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions{};

		// pos 속성 정보 입력
		attributeDescriptions[0].binding = 0;  // 버텍스 버퍼의 바인딩 포인트
		attributeDescriptions[0].location = 0; // 버텍스 셰이더의 어떤 location에 대응되는지 지정
		attributeDescriptions[0].format =
			VK_FORMAT_R32G32B32_SFLOAT; // 저장되는 데이터 형식 (VK_FORMAT_R32G32B32_SFLOAT = vec3)
		attributeDescriptions[0].offset = offsetof(Vertex, position); // 버텍스 구조체에서 해당 속성이 시작되는 위치

		// color 속성 정보 입력
		attributeDescriptions[1].binding = 0;
		attributeDescriptions[1].location = 1;
		attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[1].offset = offsetof(Vertex, normal);

		// texCoord 속성 정보 입력
		attributeDescriptions[2].binding = 0;
		attributeDescriptions[2].location = 2;
		attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
		attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

		return attributeDescriptions;
	}
};

#endif