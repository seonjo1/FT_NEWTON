#include "../include/syncObject.h"

std::unique_ptr<SyncObject> SyncObject::create(VkDevice device)
{
	std::unique_ptr<SyncObject> syncObject(new SyncObject());
	syncObject->init(device);
	return syncObject;
}


void SyncObject::init(VkDevice device)
{
	this->device = device;
	createSyncObjects();
}

void SyncObject::clear()
{
	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
		vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
		vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
		vkDestroyFence(device, inFlightFences[i], nullptr);
	}
}

VkSemaphore SyncObject::getImageAvailableSemaphore(uint32_t currentFrame)
{
	return imageAvailableSemaphores[currentFrame];
}

VkSemaphore SyncObject::getRenderFinishedSemaphore(uint32_t currentFrame)
{
	return renderFinishedSemaphores[currentFrame];
}

VkFence SyncObject::getInFlightFence(uint32_t currentFrame)
{
	return inFlightFences[currentFrame];
}

VkFence* SyncObject::getInFlightFencePointer(uint32_t currentFrame)
{
	return &inFlightFences[currentFrame];
}

/*
	[동기화 오브젝트 생성]
	세마포어 - GPU, GPU 작업간 동기화
	펜스 - CPU, GPU 작업간 동기화
*/
void SyncObject::createSyncObjects()
{
	// 세마포어, 펜스 vector 동시에 처리할 최대 프레임 버퍼 수만큼 할당
	imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
	renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
	inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

	// 세마포어 생성 설정 값 준비
	VkSemaphoreCreateInfo semaphoreInfo{};
	semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

	// 펜스 생성 설정 값 준비
	VkFenceCreateInfo fenceInfo{};
	fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;         // signal 등록된 상태로 생성 (시작하자마자 wait으로 시작하므로 필요한 FLAG)

	// 세마포어, 펜스 생성
	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
		if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
			vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
			vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS) {
			throw std::runtime_error("failed to create synchronization objects for a frame!");
		}
	}
}