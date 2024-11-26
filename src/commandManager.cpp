#include "../include/commandManager.h"

std::unique_ptr<CommandManager> CommandManager::create(DeviceManager* deviceManager)
{
	std::unique_ptr<CommandManager> commandManager(new CommandManager());
	commandManager->init(deviceManager);
	return commandManager;
}

void CommandManager::init(DeviceManager* deviceManager)
{
	device = deviceManager->getLogicalDevice();
	createCommandPool(deviceManager->getQueueFamilyIndices());
	createCommandBuffers();
}

void CommandManager::clear()
{
	vkDestroyCommandPool(device, commandPool, nullptr);
}

VkCommandPool CommandManager::getCommandPool()
{
	return commandPool;
}

VkCommandBuffer CommandManager::getCommandBuffer(uint32_t currentFrame)
{
	return commandBuffers[currentFrame];
}

VkCommandBuffer* CommandManager::getCommandBufferPointer(uint32_t currentFrame)
{
	return &commandBuffers[currentFrame];
}

/*
	[커맨드 풀 생성]
	커맨드 풀이란?
	1. 커맨드 버퍼들을 관리한다.
	2. 큐 패밀리당 1개의 커맨드 풀이 필요하다.
*/
void CommandManager::createCommandPool(QueueFamilyIndices queueFamilyIndices) {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT; 		// 커맨드 버퍼를 개별적으로 재설정할 수 있도록 설정 
																			// (이게 아니면 커맨드 풀의 모든 커맨드 버퍼 설정이 한 번에 이루어짐)
	poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value(); 	// 그래픽스 큐 인덱스 등록 (대응시킬 큐 패밀리 등록)

	// 커맨드 풀 생성
	if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS) {
		throw std::runtime_error("failed to create command pool!");
	}
}

/*
	[커맨드 버퍼 생성]
	커맨드 버퍼에 GPU에서 실행할 작업을 전부 기록한뒤 제출한다.
	GPU는 해당 커맨드 버퍼의 작업을 알아서 실행하고, CPU는 다른 일을 할 수 있게 된다. (병렬 처리)
*/
void CommandManager::createCommandBuffers() {
	// 동시에 처리할 프레임 버퍼 수만큼 커맨드 버퍼 생성
	commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

	// 커맨드 버퍼 설정값 준비
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.commandPool = commandPool; 								// 커맨드 풀 등록
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;					// 큐에 직접 제출할 수 있는 커맨드 버퍼 설정
	allocInfo.commandBufferCount = (uint32_t) commandBuffers.size(); 	// 할당할 커맨드 버퍼의 개수

	// 커맨드 버퍼 할당
	if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) {
		throw std::runtime_error("failed to allocate command buffers!");
	}
}