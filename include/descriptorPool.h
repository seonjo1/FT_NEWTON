#ifndef DESCRIPTORPOOL_H
# define DESCRIPTORPOOL_H

# include "model.h"

class DescriptorPool
{
public:
	static std::unique_ptr<DescriptorPool> create(VkDevice device, std::vector<std::unique_ptr<Model>>& models);
	~DescriptorPool() = default;
	void clear();
	VkDescriptorPool get();

private:
	DescriptorPool() = default;
	void init(VkDevice device, std::vector<std::unique_ptr<Model>>& models);
	void createDescriptorPool( std::vector<std::unique_ptr<Model>>& models);

	VkDevice device;
	VkDescriptorPool descriptorPool;
};

#endif