#include "../include/DeviceManager.h"

std::unique_ptr<DeviceManager> DeviceManager::create(VulkanInstance *vulkanInstance)
{
	std::unique_ptr<DeviceManager> deviceManager(new DeviceManager());
	deviceManager->init(vulkanInstance);
	return deviceManager;
}

void DeviceManager::init(VulkanInstance *vulkanInstance)
{
	pickPhysicalDevice(vulkanInstance);
	createLogicalDevice();
}

// 적절한 GPU 고르는 함수
void DeviceManager::pickPhysicalDevice(VulkanInstance *vulkanInstance)
{

	// GPU 장치 목록 불러오기
	uint32_t deviceCount = 0;
	vkEnumeratePhysicalDevices(vulkanInstance->getInstance(), &deviceCount, nullptr);

	if (deviceCount == 0)
	{
		throw std::runtime_error("failed to find GPUs with Vulkan support!");
	}

	std::vector<VkPhysicalDevice> devices(deviceCount);
	vkEnumeratePhysicalDevices(vulkanInstance->getInstance(), &deviceCount, devices.data());

	// 적합한 GPU 탐색
	for (const auto &device : devices)
	{
		if (isDeviceSuitable(device, vulkanInstance->getSurface()))
		{
			physicalDevice = device;
			msaaSamples = getMaxUsableSampleCount();
			break;
		}
	}

	// 적합한 GPU가 발견되지 않은 경우 에러 발생
	if (physicalDevice == VK_NULL_HANDLE)
	{
		throw std::runtime_error("failed to find a suitable GPU!");
	}
}

// 그래픽, 프레젠테이션 큐 패밀리가 존재하는지, GPU와 surface가 호환하는 SwapChain이 존재하는지 검사
bool DeviceManager::isDeviceSuitable(VkPhysicalDevice device, VkSurfaceKHR surface)
{
	// 큐 패밀리 확인
	queueFamilyIndices = findQueueFamilies(device, surface);

	// 스왑 체인 확장을 지원하는지 확인
	bool extensionsSupported = checkDeviceExtensionSupport(device);
	bool swapChainAdequate = false;

	// 물리 디바이스와 surface가 호환하는 SwapChain 정보를 가져옴
	swapChainSupport = querySwapChainSupport(device, surface);
	// 스왑 체인 확장이 존재하는 경우
	if (extensionsSupported)
	{
		// GPU와 surface가 지원하는 format과 presentMode가 존재하면 통과
		swapChainAdequate = !swapChainSupport.formats.empty() && !swapChainSupport.presentModes.empty();
	}

	// GPU 에서 이방성 필터링을 지원하는지 확인
	VkPhysicalDeviceFeatures supportedFeatures;
	vkGetPhysicalDeviceFeatures(device, &supportedFeatures);

	return queueFamilyIndices.isComplete() && extensionsSupported && swapChainAdequate &&
		   supportedFeatures.samplerAnisotropy;
}

// 디바이스가 지원하는 확장 중
bool DeviceManager::checkDeviceExtensionSupport(VkPhysicalDevice device)
{
	uint32_t extensionCount;
	vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

	std::vector<VkExtensionProperties> availableExtensions(extensionCount);
	vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

	// 스왑 체인 확장이 존재하는지 확인
	std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());
	for (const auto &extension : availableExtensions)
	{
		// 지원 가능한 확장들 목록을 순회하며 제거
		requiredExtensions.erase(extension.extensionName);
	}

	// 만약 기존에 있던 확장이 제거되면 true
	// 기존에 있던 확장이 그대로면 지원을 안 하는 것이므로 false
	return requiredExtensions.empty();
}

// GPU와 surface가 호환하는 SwapChain 정보를 반환
SwapChainSupportDetails DeviceManager::querySwapChainSupport(VkPhysicalDevice device, VkSurfaceKHR surface)
{
	SwapChainSupportDetails details;

	// GPU와 surface가 호환할 수 있는 capability 정보 쿼리
	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &details.capabilities);

	// device에서 surface 객체를 지원하는 format이 존재하는지 확인
	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);

	if (formatCount != 0)
	{
		details.formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, details.formats.data());
	}

	// device에서 surface 객체를 지원하는 presentMode가 있는지 확인
	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);

	if (presentModeCount != 0)
	{
		details.presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, details.presentModes.data());
	}

	return details;
}

QueueFamilyIndices DeviceManager::findQueueFamilies(VkPhysicalDevice device, VkSurfaceKHR surface)
{
	QueueFamilyIndices indices;

	// GPU가 지원하는 큐 패밀리 개수 가져오기
	uint32_t queueFamilyCount = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

	// GPU가 지원하는 큐 패밀리 리스트 가져오기
	std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
	vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

	// 그래픽 큐 패밀리 검색
	int i = 0;
	for (const auto &queueFamily : queueFamilies)
	{
		// 그래픽 큐 패밀리 찾기 성공한 경우 indices에 값 생성
		if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT)
		{
			indices.graphicsFamily = i;
		}

		// GPU의 i 인덱스 큐 패밀리가 surface에서 프레젠테이션을 지원하는지 확인
		VkBool32 presentSupport = false;
		vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);

		// 프레젠테이션 큐 패밀리 등록
		if (presentSupport)
		{
			indices.presentFamily = i;
		}

		// 그래픽 큐 패밀리 찾은 경우 break
		if (indices.isComplete())
		{
			break;
		}

		i++;
	}
	// 그래픽 큐 패밀리를 못 찾은 경우 값이 없는 채로 반환 됨
	return indices;
}

// GPU 에서 지원하는 최대 샘플 개수 반환
VkSampleCountFlagBits DeviceManager::getMaxUsableSampleCount()
{
	VkPhysicalDeviceProperties physicalDeviceProperties;
	vkGetPhysicalDeviceProperties(physicalDevice, &physicalDeviceProperties);

	// GPU가 지원하는 color 샘플링 개수와 depth 샘플링 개수의 공통 분모를 찾음
	VkSampleCountFlags counts = physicalDeviceProperties.limits.framebufferColorSampleCounts &
								physicalDeviceProperties.limits.framebufferDepthSampleCounts;

	// 가장 높은 샘플링 개수부터 확인하면서 반환
	if (counts & VK_SAMPLE_COUNT_64_BIT)
	{
		return VK_SAMPLE_COUNT_64_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_32_BIT)
	{
		return VK_SAMPLE_COUNT_32_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_16_BIT)
	{
		return VK_SAMPLE_COUNT_16_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_8_BIT)
	{
		return VK_SAMPLE_COUNT_8_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_4_BIT)
	{
		return VK_SAMPLE_COUNT_4_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_2_BIT)
	{
		return VK_SAMPLE_COUNT_2_BIT;
	}

	return VK_SAMPLE_COUNT_1_BIT;
}

// GPU와 소통할 인터페이스인 Logical device 생성
void DeviceManager::createLogicalDevice()
{
	// 큐 패밀리의 인덱스들을 set으로 래핑
	std::set<uint32_t> uniqueQueueFamilies = {queueFamilyIndices.graphicsFamily.value(),
											  queueFamilyIndices.presentFamily.value()};

	// 큐 생성을 위한 정보 설정
	std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
	// 큐 우선순위 0.0f ~ 1.0f 로 표현
	float queuePriority = 1.0f;
	// 큐 패밀리 별로 정보 생성
	for (uint32_t queueFamily : uniqueQueueFamilies)
	{
		VkDeviceQueueCreateInfo queueCreateInfo{};
		queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queueCreateInfo.queueFamilyIndex = queueFamily;
		queueCreateInfo.queueCount = 1;
		queueCreateInfo.pQueuePriorities = &queuePriority;
		queueCreateInfos.push_back(queueCreateInfo);
	}

	// 사용할 장치 기능이 포함된 구조체
	// vkGetPhysicalDeviceFeatures 함수로 디바이스에서 설정 가능한
	// 장치 기능 목록을 확인할 수 있음
	// 일단 지금은 VK_FALSE로 전부 등록함
	VkPhysicalDeviceFeatures deviceFeatures{};
	deviceFeatures.samplerAnisotropy = VK_TRUE; // 이방성 필터링 사용 설정
	deviceFeatures.sampleRateShading = VK_TRUE; // 디바이스에 샘플 셰이딩 기능 활성화

	// 논리적 장치 생성을 위한 정보 등록
	VkDeviceCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
	createInfo.pQueueCreateInfos = queueCreateInfos.data();
	createInfo.pEnabledFeatures = &deviceFeatures;

	// 확장 설정
	createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
	createInfo.ppEnabledExtensionNames = deviceExtensions.data();

	// 구버전 호환을 위해 디버그 모드일 경우
	// 검증 레이어를 포함 시키지만, 현대 시스템에서는 논리적 장치의 레이어를 안 씀
	if (enableValidationLayers)
	{
		createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
		createInfo.ppEnabledLayerNames = validationLayers.data();
	}
	else
	{
		createInfo.enabledLayerCount = 0;
	}

	// 논리적 장치 생성
	if (vkCreateDevice(physicalDevice, &createInfo, nullptr, &device) != VK_SUCCESS)
	{
		throw std::runtime_error("failed to create logical device!");
	}

	// 큐 핸들 가져오기
	vkGetDeviceQueue(device, queueFamilyIndices.graphicsFamily.value(), 0, &graphicsQueue);
	vkGetDeviceQueue(device, queueFamilyIndices.presentFamily.value(), 0, &presentQueue);
}

// Vulkan의 특정 format에 대해 GPU가 tiling의 features를 지원하는지 확인
VkFormat DeviceManager::findSupportedFormat(const std::vector<VkFormat> &candidates, VkImageTiling tiling,
											VkFormatFeatureFlags features)
{
	// format 들에 대해 GPU가 tiling과 features를 지원하는지 확인
	for (VkFormat format : candidates)
	{
		// GPU가 format에 대해 지원하는 특성 가져오는 함수
		VkFormatProperties props;
		vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);

		// GPU가 지원하는 특성과 비교
		if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features)
		{ // VK_IMAGE_TILING_LINEAR의 특성 비교
			return format;
		}
		else if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features)
		{ // VK_IMAGE_TILING_OPTIMAL의 특성 비교
			return format;
		}
	}

	throw std::runtime_error("failed to find supported format!");
}

VkFormat DeviceManager::findDepthFormat()
{
	return findSupportedFormat({VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT},
							   VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
}

void DeviceManager::clear()
{
	vkDestroyDevice(device, nullptr); // 논리적 장치 파괴
}

VkPhysicalDevice DeviceManager::getPhysicalDevice()
{
	return physicalDevice;
}

VkDevice DeviceManager::getLogicalDevice()
{
	return device;
}

QueueFamilyIndices DeviceManager::getQueueFamilyIndices()
{
	return queueFamilyIndices;
}

VkQueue DeviceManager::getGraphicsQueue()
{
	return graphicsQueue;
}

VkQueue DeviceManager::getPresentQueue()
{
	return presentQueue;
}

VkSampleCountFlagBits DeviceManager::getMsaaSamples()
{
	return msaaSamples;
}

SwapChainSupportDetails DeviceManager::getSwapChainSupport(VkSurfaceKHR surface)
{
	// GPU와 surface가 호환할 수 있는 capability 정보 쿼리
	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &swapChainSupport.capabilities);

	// device에서 surface 객체를 지원하는 format이 존재하는지 확인
	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);

	if (formatCount != 0)
	{
		swapChainSupport.formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, swapChainSupport.formats.data());
	}

	// device에서 surface 객체를 지원하는 presentMode가 있는지 확인
	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, nullptr);

	if (presentModeCount != 0)
	{
		swapChainSupport.presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount,
												  swapChainSupport.presentModes.data());
	}

	return swapChainSupport;
}
