#ifndef COMMANDMANAGER_H
#define COMMANDMANAGER_H

#include "DeviceManager.h"

class CommandManager
{
  public:
	static std::unique_ptr<CommandManager> create(DeviceManager *deviceManager);
	~CommandManager() = default;
	void clear();
	VkCommandPool getCommandPool();
	VkCommandBuffer getCommandBuffer(uint32_t currentFrame);
	VkCommandBuffer *getCommandBufferPointer(uint32_t currentFrame);

  private:
	CommandManager() = default;
	void init(DeviceManager *deviceManager);
	void createCommandPool(QueueFamilyIndices queueFamilyIndices);
	void createCommandBuffers();

	VkDevice device;
	VkCommandPool commandPool;
	std::vector<VkCommandBuffer> commandBuffers;
};

#endif