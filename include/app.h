#ifndef APP_H
#define APP_H

#include "Camera.h"
#include "CommandManager.h"
#include "DescriptorPool.h"
#include "Image.h"
#include "physics/World.h"
#include "Renderer.h"
#include "SwapChainManager.h"
#include "SyncObject.h"

class App
{
  public:
	void run();

	App() = default;
	~App() = default;

  public:
	const ale::Transform &getTransformById(int32_t xfId) const;
	void setTransformById(int32_t xfId, const ale::Transform &xf);
	void calculateTransformMatrix(int32_t xfId, glm::mat4 &transformMatrix);

  private:
	void initWindow();
	void initVulkan();
	void mainLoop();
	void cleanup();

	void createModels();
	void createWorld();

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

	// event process 함수
	void processEvents();

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
	std::vector<ale::Transform> transforms;

	Camera camera;
	bool cameraControl{false};

	bool shootControl{false};
	bool canShoot{true};
	int32_t shootNum{0};
	const int32_t SHOOT_MAX{100};

	std::unique_ptr<ale::World> world;

	uint32_t currentFrame = 0;
	bool framebufferResized = false;
	const uint32_t WINDOW_WIDTH = 1920;
	const uint32_t WINDOW_HEIGHT = 1080;
};

#endif