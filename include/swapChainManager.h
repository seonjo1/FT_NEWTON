#ifndef SWAPCHAINMANAGER_H
# define SWAPCHAINMANAGER_H

# include "common.h"
# include "vulkanInstance.h"
# include "deviceManager.h"

# include <algorithm>
# include <array>

class ColorImage;
class DepthImage;

class SwapChainManager
{
public:
	static std::unique_ptr<SwapChainManager> create(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface);
	~SwapChainManager() = default;
	void createFramebuffers(DeviceManager* deviceManager, VkRenderPass renderPass);
	void recreateSwapChain(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface, VkRenderPass renderPass);
	void cleanupSwapChain();
	VkSwapchainKHR getSwapChain();
	VkFormat getSwapChainImageFormat();
	VkExtent2D getSwapChainExtent();
	std::vector<VkFramebuffer>& getFramebuffers();

private:
	SwapChainManager() = default;
	void init(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface);
	void createSwapChain(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface);
	void createImageViews();
	VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
	VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
	VkExtent2D chooseSwapExtent(GLFWwindow* window, const VkSurfaceCapabilitiesKHR& capabilities);
	
	void createColorResources(DeviceManager* deviceManager);
	void createDepthResources(DeviceManager* deviceManager);
	VkFormat findSupportedFormat(VkPhysicalDevice physicalDevice, const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features);
	void findDepthFormat(VkPhysicalDevice physicalDevice);
	void createImage(VkPhysicalDevice physicalDevice, uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory);
	VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels);

	VkDevice device;
	VkSwapchainKHR swapChain;
	std::vector<VkImage> swapChainImages;
	std::vector<VkImageView> swapChainImageViews;
	VkFormat swapChainImageFormat;
	VkExtent2D swapChainExtent;

	std::unique_ptr<ColorImage> colorImage;
	std::unique_ptr<DepthImage> depthImage;
	std::vector<VkFramebuffer> swapChainFramebuffers;
};

#endif