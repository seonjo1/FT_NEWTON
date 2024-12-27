#ifndef COMMON_H
#define COMMON_H

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <array>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

// 디버그 모드시 검증 레이어 사용
#ifdef NDEBUG
const bool enableValidationLayers = false;
#else
const bool enableValidationLayers = true;
#endif

// 검증 레이어 설정
const std::vector<const char *> validationLayers = {"VK_LAYER_KHRONOS_validation"};

// 동시에 처리할 최대 프레임 수
const int MAX_FRAMES_IN_FLIGHT = 2;

// GPU와 surface가 지원하는 SwapChain 지원 세부 정보 구조체
struct SwapChainSupportDetails
{
	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> presentModes;
};

// shader 파일인 SPIR-V 파일을 바이너리 형태로 읽어오는 함수
static std::vector<char> readFile(const std::string &filename)
{
	std::ifstream file(filename, std::ios::ate | std::ios::binary);

	if (!file.is_open())
	{
		throw std::runtime_error("failed to open file!");
	}

	size_t fileSize = (size_t)file.tellg();
	std::vector<char> buffer(fileSize);

	file.seekg(0);
	file.read(buffer.data(), fileSize);

	file.close();

	return buffer;
}

struct UniformBufferObject
{
	alignas(16) glm::mat4 model;
	alignas(16) glm::mat4 view;
	alignas(16) glm::mat4 proj;
	alignas(16) int32_t isCollide;
};

#endif
