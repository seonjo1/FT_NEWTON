#ifndef DESCRIPTORPOOL_H
#define DESCRIPTORPOOL_H

#include "Model.h"

class DescriptorPool
{
  public:
	static std::unique_ptr<DescriptorPool> create(VkDevice device, std::vector<std::unique_ptr<Model>> &models,
												  int32_t shootMax);
	~DescriptorPool() = default;
	void clear();
	VkDescriptorPool get();

  private:
	DescriptorPool() = default;
	void init(VkDevice device, std::vector<std::unique_ptr<Model>> &models, int32_t shootMax);
	void createDescriptorPool(std::vector<std::unique_ptr<Model>> &models, int32_t shootMax);

	VkDevice device;
	VkDescriptorPool descriptorPool;
};

#endif