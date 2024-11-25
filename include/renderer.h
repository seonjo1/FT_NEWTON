#ifndef RENDERER_H
# define RENDERER_H

# include "common.h"
# include "deviceManager.h"

class Renderer
{
public:
	static std::unique_ptr<Renderer> create(DeviceManager* deviceManager, VkFormat swapChainImageFormat);
	~Renderer() = default;
	void clear();
	VkRenderPass getRenderPass();
	VkDescriptorSetLayout getDescriptorSetLayout();
	VkPipelineLayout getPipelineLayout();
	VkPipeline getGraphicsPipeline();

private:
	Renderer() = default;
	void init(DeviceManager* deviceManager, VkFormat swapChainImageFormat);
	void createRenderPass(DeviceManager* deviceManager, VkFormat swapChainImageFormat);
	void createDescriptorSetLayout();
	void createGraphicsPipeline(VkSampleCountFlagBits msaaSamples);
	VkShaderModule createShaderModule(const std::vector<char>& code);


	VkDevice device;
	VkRenderPass renderPass;
	VkDescriptorSetLayout descriptorSetLayout;
	VkPipelineLayout pipelineLayout;
	VkPipeline graphicsPipeline;
};

#endif