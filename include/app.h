#ifndef APP_H
#define APP_H

#include "camera.h"
#include "commandManager.h"
#include "descriptorPool.h"
#include "image.h"
#include "renderer.h"
#include "swapChainManager.h"
#include "syncObject.h"
#include "world.h"

class App
{
  public:
	void run();

	App() = default;
	~App() = default;

  private:
	void initWindow();
	void initVulkan();
	void mainLoop();
	void cleanup();

	void createModels();
	void initWorld();

	// draw 함수
	void drawFrame();
	bool tryPrepareImage(uint32_t *imageIndex);
	void updateUniformBuffer();
	void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
	void submitRenderingCommandBuffer();
	void submitPresentationCommandBuffer(uint32_t imageIndex);

	// 콜백 함수
	static void framebufferResizeCallback(GLFWwindow *window, int width, int height);
	static void cursorPosCallback(GLFWwindow *window, double x, double y);
	static void mouseButtonCallback(GLFWwindow *window, int button, int action, int modifier);
	static void keyEventCallback(GLFWwindow *window, int key, int scancode, int action, int mods);

	// camera contorl 함수
	void processCameraControl();
	void mouseMove(double x, double y);
	void mouseButton(int button, int action, double x, double y);

	GLFWwindow *window;
	VkDevice device;

	std::unique_ptr<VulkanInstance> vulkanInstance;
	std::unique_ptr<DeviceManager> deviceManager;
	std::unique_ptr<SwapChainManager> swapChainManager;
	std::unique_ptr<CommandManager> commandManager;
	std::unique_ptr<DescriptorPool> descriptorPool;
	std::unique_ptr<SyncObject> syncObject;
	std::unique_ptr<Renderer> renderer;

	std::vector<std::unique_ptr<Model>> models;

	Camera camera;
	bool cameraControl{false};

	std::unique_ptr<ale::World> *world;

	uint32_t currentFrame = 0;
	bool framebufferResized = false;
};

#endif