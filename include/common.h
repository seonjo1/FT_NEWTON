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

#endif
