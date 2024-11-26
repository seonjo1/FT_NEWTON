#ifndef APP_H
# define APP_H

# include "swapChainManager.h"
# include "image.h"
# include "renderer.h"
# include "model.h"

class App {
public:
	void run();

private:
	void initWindow();
	void initVulkan();
	void mainLoop();
	void cleanup();
	void drawFrame();
	void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
	void createCommandPool();
	void createCommandBuffers();
	void createDescriptorPool();
	void createSyncObjects();
	static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
	
	GLFWwindow* window;
	VkDevice device;
	std::unique_ptr<VulkanInstance> vulkanInstance;
	std::unique_ptr<DeviceManager> deviceManager;
	std::unique_ptr<SwapChainManager> swapChainManager;
	std::unique_ptr<Renderer> renderer;
	VkCommandPool commandPool;
	std::unique_ptr<Model> model;
	VkDescriptorPool descriptorPool;
	std::vector<VkCommandBuffer> commandBuffers;
	std::vector<VkSemaphore> imageAvailableSemaphores;
	std::vector<VkSemaphore> renderFinishedSemaphores;
	std::vector<VkFence> inFlightFences;
	uint32_t currentFrame = 0;
	bool framebufferResized = false;
};

#endif 