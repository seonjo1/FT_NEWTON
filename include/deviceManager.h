#ifndef DEVICEMANAGER_H
# define DEVICEMANAGER_H

# include "vulkanInstance.h"

# include <optional>
# include <set>

// 큐 패밀리 인덱스 관리 구조체
struct QueueFamilyIndices {
	std::optional<uint32_t> graphicsFamily;
	std::optional<uint32_t> presentFamily;

	bool isComplete() {
		return graphicsFamily.has_value() && presentFamily.has_value();
	}
};

class DeviceManager
{
public:
	static std::unique_ptr<DeviceManager> create(VulkanInstance* vulkanInstance);
	~DeviceManager() = default;
	void clear();
	VkPhysicalDevice getPhysicalDevice();
	VkDevice getLogicalDevice();
	QueueFamilyIndices getQueueFamilyIndices();
	VkQueue getGraphicsQueue();
	VkQueue getPresentQueue();
	VkSampleCountFlagBits getMsaaSamples();
	SwapChainSupportDetails getSwapChainSupport(VkSurfaceKHR surface);
	VkFormat findDepthFormat();

private:
	DeviceManager() = default;
	void init(VulkanInstance* vulkanInstance);
	void pickPhysicalDevice(VulkanInstance* vulkanInstance);
	void createLogicalDevice();
	bool isDeviceSuitable(VkPhysicalDevice device, VkSurfaceKHR surface);
	bool checkDeviceExtensionSupport(VkPhysicalDevice device);
	SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device, VkSurfaceKHR surface);
	QueueFamilyIndices  findQueueFamilies(VkPhysicalDevice device, VkSurfaceKHR surface);
	VkSampleCountFlagBits getMaxUsableSampleCount();
	VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features);

	VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
	VkDevice device;
	VkQueue graphicsQueue;
	VkQueue presentQueue;
	QueueFamilyIndices queueFamilyIndices;
	SwapChainSupportDetails swapChainSupport;
	VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT;
	const std::vector<const char*> deviceExtensions = {	
		VK_KHR_SWAPCHAIN_EXTENSION_NAME		// 스왑 체인 확장
	};

};

#endif