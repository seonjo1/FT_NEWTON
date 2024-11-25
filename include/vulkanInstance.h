#ifndef VULKANINSTANCE_H
# define VULKANINSTANCE_H

# include "common.h"

class VulkanInstance
{
public:
	static std::unique_ptr<VulkanInstance> create(GLFWwindow* window);
	~VulkanInstance() = default;
	void clear();
	VkInstance getInstance();
	VkSurfaceKHR getSurface();

private:
	VulkanInstance() = default;
	void init(GLFWwindow* window);
	void createInstance();
	void setupDebugMessenger();
	void createSurface(GLFWwindow* window);
	bool checkValidationLayerSupport();
	std::vector<const char*> getRequiredExtensions();
	void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
	VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
											const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger);
	void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator);
	static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback( VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
														VkDebugUtilsMessageTypeFlagsEXT messageType,
														const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
														void* pUserData);

	VkInstance instance;
	VkDebugUtilsMessengerEXT debugMessenger;
	VkSurfaceKHR surface;

};

#endif