#include "../include/DescriptorPool.h"

std::unique_ptr<DescriptorPool> DescriptorPool::create(VkDevice device, std::vector<std::unique_ptr<Model>>& models, int32_t shootMax)
{
	std::unique_ptr<DescriptorPool> descriptorPool(new DescriptorPool());
	descriptorPool->init(device, models, shootMax);
	return descriptorPool;
}

void DescriptorPool::init(VkDevice device, std::vector<std::unique_ptr<Model>>& models, int32_t shootMax)
{
	this->device = device;
	createDescriptorPool(models, shootMax);
}

void DescriptorPool::clear()
{
	vkDestroyDescriptorPool(device, descriptorPool, nullptr);
}

VkDescriptorPool DescriptorPool::get()
{
	return descriptorPool;
}

void DescriptorPool::createDescriptorPool( std::vector<std::unique_ptr<Model>>& models, int32_t shootMax)
{

	uint32_t meshNum = shootMax;

	for (std::unique_ptr<Model>& model : models)
	{
		meshNum += model->getSize();
	}

	// 디스크립터 풀의 타입별 디스크립터 개수를 설정하는 구조체
	std::array<VkDescriptorPoolSize, 2> poolSizes{};
	poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;												// 유니폼 버퍼 설정
	poolSizes[0].descriptorCount = static_cast<uint32_t>(meshNum * MAX_FRAMES_IN_FLIGHT);		// 유니폼 버퍼 디스크립터 최대 개수 설정
	poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;										// 샘플러 설정
	poolSizes[1].descriptorCount = static_cast<uint32_t>(2 * meshNum * MAX_FRAMES_IN_FLIGHT);	// 샘플러 디스크립터 최대 개수 설정

	// 디스크립터 풀을 생성할 때 필요한 설정 정보를 담는 구조체
	VkDescriptorPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());			// 디스크립터 poolSize 구조체 개수
	poolInfo.pPoolSizes = poolSizes.data();										// 디스크립터 poolSize 구조체 배열
	poolInfo.maxSets = static_cast<uint32_t>(meshNum * MAX_FRAMES_IN_FLIGHT);				// 풀에 존재할 수 있는 총 디스크립터 셋 개수

	// 디스크립터 풀 생성
	if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
		throw std::runtime_error("failed to create descriptor pool!");
	}
}