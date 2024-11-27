#ifndef MODEL_H
# define MODEL_H

# include "mesh.h"

class Model
{
public:
	static std::unique_ptr<Model> create(std::string filename, DeviceManager* deviceManager, VkCommandPool commandPool);
	void clear();
	void recordDrawCommand(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, uint32_t currentFrame);
	void updateUniformBuffer(UniformBufferObject& ubo, uint32_t currentFrame);
	void createDescriptorSets(VkDevice device, VkDescriptorPool descriptorPool, VkDescriptorSetLayout descriptorSetLayout);
	uint32_t getSize();

private:
	void load(std::string filename, DeviceManager* deviceManager, VkCommandPool commandPool);
	void processNode(DeviceManager* deviceManager, VkCommandPool commandPool, aiNode *node, const aiScene *scene);
	void processMesh(DeviceManager* deviceManager, VkCommandPool commandPool, aiMesh *mesh, const aiScene *scene);
	
	std::vector<std::unique_ptr<Mesh>> meshes;
	std::vector<std::unique_ptr<Material>> materials;
	uint32_t size;
};

#endif