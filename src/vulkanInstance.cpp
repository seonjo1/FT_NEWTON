#include "../include/vulkanInstance.h"

std::unique_ptr<VulkanInstance> VulkanInstance::create(GLFWwindow* window)
{
	std::unique_ptr<VulkanInstance> vulkanInstance(new VulkanInstance());
	vulkanInstance->init(window);
	return vulkanInstance;
}

void VulkanInstance::init(GLFWwindow* window)
{
	createInstance();
	setupDebugMessenger();
	createSurface(window);
}

/*
	[인스턴스 생성]
	인스턴스 구성 요소
	1. 애플리케이션 정보
	2. 확장
	3. 레이어
	4. 디버그 모드일시 디버그 메신저 객체 생성 정보 추가
*/ 
void VulkanInstance::createInstance() {
	// 디버그 모드에서 검증 레이어 적용 불가능시 예외 발생
	if (enableValidationLayers && !checkValidationLayerSupport()) {
		throw std::runtime_error("validation layers requested, but not available!");
	}

	// 애플리케이션 정보를 담은 구조체
	VkApplicationInfo appInfo{};
	appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pApplicationName = "Hello Triangle";
	appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.pEngineName = "No Engine";
	appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.apiVersion = VK_API_VERSION_1_0;

	// 인스턴스 생성을 위한 정보를 담은 구조체
	VkInstanceCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	createInfo.pApplicationInfo = &appInfo;

	std::vector<const char*> extensions = getRequiredExtensions();
	createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
	createInfo.ppEnabledExtensionNames = extensions.data();

	// 디버깅 메시지 객체 생성을 위한 정보 구조체
	VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo{};

	if (enableValidationLayers) {
		// 디버그 모드시 구조체에 검증 레이어 포함
		createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
		createInfo.ppEnabledLayerNames = validationLayers.data();
		// 인스턴스 생성 및 파괴시에도 검증 가능
		populateDebugMessengerCreateInfo(debugCreateInfo);
		createInfo.pNext = reinterpret_cast<VkDebugUtilsMessengerCreateInfoEXT*>(&debugCreateInfo);
	} else {
		// 디버그 모드 아닐 시 검증 레이어 x
		createInfo.enabledLayerCount = 0;		
		createInfo.pNext = nullptr;
	}

	// 인스턴스 생성
	if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
		throw std::runtime_error("failed to create instance!");
	}
}

// 디버그 메신저 객체 생성
void VulkanInstance::setupDebugMessenger() {
	// 디버그 모드 아니면 return
	if (!enableValidationLayers) return;

	// VkDebugUtilsMessengerCreateInfoEXT 구조체 생성
	VkDebugUtilsMessengerCreateInfoEXT createInfo;
	populateDebugMessengerCreateInfo(createInfo);

	// 디버그 메시지 객체 생성
	if (CreateDebugUtilsMessengerEXT(instance, &createInfo, nullptr, &debugMessenger) != VK_SUCCESS) {
		throw std::runtime_error("failed to set up debug messenger!");
	}
}

// OS에 맞는 surface를 glfw 함수를 통해 생성
void VulkanInstance::createSurface(GLFWwindow* window) {
	if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
		throw std::runtime_error("failed to create window surface!");
	}
}

// 검증 레이어가 사용 가능한 레이어 목록에 있는지 확인
bool VulkanInstance::checkValidationLayerSupport() {
	// Vulkan 인스턴스에서 사용 가능한 레이어들 목록 생성
	uint32_t layerCount;
	vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
	std::vector<VkLayerProperties> availableLayers(layerCount);
	vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

	// 필요한 검증 레이어들이 사용 가능 레이어에 포함 되어있는지 확인
	for (const char* layerName : validationLayers) {
		bool layerFound = false;
		for (const auto& layerProperties : availableLayers) {
			if (strcmp(layerName, layerProperties.layerName) == 0) {
				// 포함 확인!
				layerFound = true;
				break;
			}
		}
		if (!layerFound) {
			return false;  // 필요한 레이어가 없다면 false 반환
		}
	}
	return true;  // 모든 레이어가 지원되면 true 반환
}

/*
GLFW 라이브러리에서 Vulkan 인스턴스를 생성할 때 필요한 인스턴스 확장 목록을 반환
(디버깅 모드시 메시지 콜백 확장 추가)
*/
std::vector<const char*> VulkanInstance::getRequiredExtensions() {
	// 필요한 확장 목록 가져오기
	uint32_t glfwExtensionCount = 0;
	const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
	std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);

	// 디버깅 모드이면 VK_EXT_debug_utils 확장 추가 (메세지 콜백 확장)
	if (enableValidationLayers) {
		extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
	}
	
	return extensions;
}

//vkDebugUtilsMessengerCreateInfoEXT 구조체 내부를 채워주는 함수
void VulkanInstance::populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo) {
	createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
	createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
								VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
								VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
	createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
							VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
							VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
	createInfo.pfnUserCallback = debugCallback;
}

/*
	디버그 메신저 객체 생성 함수
	(확장 함수는 자동으로 로드 되지 않으므로 동적으로 가져와야 한다.)
*/
VkResult VulkanInstance::CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
										const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger) {
	// vkCreateDebugUtilsMessengerEXT 함수의 주소를 vkGetInstanceProcAddr를 통해 동적으로 가져옴
	PFN_vkCreateDebugUtilsMessengerEXT func = (PFN_vkCreateDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
	if (func != nullptr) {
		return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
	} else {
		return VK_ERROR_EXTENSION_NOT_PRESENT;
	}
}

/*
	디버그 메신저 객체 파괴 함수
	(확장 함수는 자동으로 로드 되지 않으므로 동적으로 가져와야 한다.)
*/
void VulkanInstance::DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
	PFN_vkDestroyDebugUtilsMessengerEXT func = (PFN_vkDestroyDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
	if (func != nullptr) {
		func(instance, debugMessenger, pAllocator);
	}
}

void VulkanInstance::clear()
{
	// 메시지 객체 파괴
	if (enableValidationLayers) {
		DestroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
	}
	vkDestroySurfaceKHR(instance, surface, nullptr);	// 화면 객체 파괴
	vkDestroyInstance(instance, nullptr);				// 인스턴스 파괴
}

// 디버그 메시지 콜백 함수
VKAPI_ATTR VkBool32 VKAPI_CALL VulkanInstance::debugCallback( VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
																VkDebugUtilsMessageTypeFlagsEXT messageType,
																const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
																void* pUserData) {
	// 메시지 내용만 출력
	std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
	// VK_TRUE 반환시 프로그램 종료됨
	return VK_FALSE;
}

VkInstance VulkanInstance::getInstance()
{
	return instance;
}

VkSurfaceKHR VulkanInstance::getSurface()
{
	return surface;
}