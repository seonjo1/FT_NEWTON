#ifndef COMMON_H
# define COMMON_H

# define GLFW_INCLUDE_VULKAN
# include <GLFW/glfw3.h>

# define GLM_FORCE_RADIANS
# define GLM_FORCE_DEPTH_ZERO_TO_ONE
# include <glm/glm.hpp>
# include <glm/gtc/matrix_transform.hpp>

# include <memory>
# include <iostream>
# include <vector>
# include <array>
# include <fstream>
# include <stdexcept>

// 디버그 모드시 검증 레이어 사용
#ifdef NDEBUG
	const bool enableValidationLayers = false;
#else
	const bool enableValidationLayers = true;
#endif

// 검증 레이어 설정
const std::vector<const char*> validationLayers = {
	"VK_LAYER_KHRONOS_validation"
};

// GPU와 surface가 지원하는 SwapChain 지원 세부 정보 구조체
struct SwapChainSupportDetails {
	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> presentModes;
};

// shader 파일인 SPIR-V 파일을 바이너리 형태로 읽어오는 함수
static std::vector<char> readFile(const std::string& filename) {
	std::ifstream file(filename, std::ios::ate | std::ios::binary);

	if (!file.is_open()) {
		throw std::runtime_error("failed to open file!");
	}

	size_t fileSize = (size_t) file.tellg();
	std::vector<char> buffer(fileSize);

	file.seekg(0);
	file.read(buffer.data(), fileSize);

	file.close();

	return buffer;
}


// renderer에 함수 의존되어있음
struct Vertex {
	glm::vec3 pos;
	glm::vec3 color;
	glm::vec2 texCoord;

	// 정점 데이터가 전달되는 방법을 알려주는 구조체 반환하는 함수
	static VkVertexInputBindingDescription getBindingDescription() {
		// 파이프라인에 정점 데이터가 전달되는 방법을 알려주는 구조체
		VkVertexInputBindingDescription bindingDescription{};		
		bindingDescription.binding = 0;								// 버텍스 바인딩 포인트 (현재 0번에 vertex 정보 바인딩)
		bindingDescription.stride = sizeof(Vertex);					// 버텍스 1개 단위의 정보 크기
		bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX; // 정점 데이터 처리 방법
																	// 1. VK_VERTEX_INPUT_RATE_VERTEX : 정점별로 데이터 처리
																	// 2. VK_VERTEX_INPUT_RATE_INSTANCE : 인스턴스별로 데이터 처리
		return bindingDescription;
	}

	// 정점 속성별 데이터 형식과 위치를 지정하는 구조체 반환하는 함수
	static std::array<VkVertexInputAttributeDescription, 3> getAttributeDescriptions() {
		// 정점 속성의 데이터 형식과 위치를 지정하는 구조체
		std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions{};

		// pos 속성 정보 입력
		attributeDescriptions[0].binding = 0;							// 버텍스 버퍼의 바인딩 포인트
		attributeDescriptions[0].location = 0;							// 버텍스 셰이더의 어떤 location에 대응되는지 지정
		attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;	// 저장되는 데이터 형식 (VK_FORMAT_R32G32B32_SFLOAT = vec3)
		attributeDescriptions[0].offset = offsetof(Vertex, pos);		// 버텍스 구조체에서 해당 속성이 시작되는 위치

		// color 속성 정보 입력
		attributeDescriptions[1].binding = 0;
		attributeDescriptions[1].location = 1;
		attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[1].offset = offsetof(Vertex, color);

		// texCoord 속성 정보 입력
		attributeDescriptions[2].binding = 0;
		attributeDescriptions[2].location = 2;
		attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
		attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

		return attributeDescriptions;
	}
};

#endif
