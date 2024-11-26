#ifndef SYNCOBJECT_H
# define SYNCOBJFECT_H

# include "common.h"

class SyncObject
{
public:
	static std::unique_ptr<SyncObject> create(VkDevice device);
	~SyncObject() = default;
	void clear();
	VkSemaphore getImageAvailableSemaphore(uint32_t currentFrame);
	VkSemaphore getRenderFinishedSemaphore(uint32_t currentFrame);
	VkFence getInFlightFence(uint32_t currentFrame);
	VkFence* getInFlightFencePointer(uint32_t currentFrame);

private:
	SyncObject() = default;
	void init(VkDevice device);
	void createSyncObjects();

	VkDevice device;
	std::vector<VkSemaphore> imageAvailableSemaphores;
	std::vector<VkSemaphore> renderFinishedSemaphores;
	std::vector<VkFence> inFlightFences;
};

#endif