#ifndef APP_H
# define APP_H

# include "swapChainManager.h"
# include "commandManager.h"
# include "image.h"
# include "renderer.h"
# include "descriptorPool.h"
# include "syncObject.h"

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

	void createDescriptorPool();
	static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
	
	GLFWwindow* window;
	VkDevice device;
	
	std::unique_ptr<VulkanInstance> vulkanInstance;
	std::unique_ptr<DeviceManager> deviceManager;
	std::unique_ptr<SwapChainManager> swapChainManager;
	std::unique_ptr<Renderer> renderer;
	std::unique_ptr<CommandManager> commandManager;
	std::vector<std::unique_ptr<Model>> models;
	std::unique_ptr<DescriptorPool> descriptorPool;
	std::unique_ptr<SyncObject> syncObject;
	uint32_t currentFrame = 0;
	bool framebufferResized = false;
};

#endif 