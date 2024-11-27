#ifndef MODEL_H
#define MODEL_H

#include "mesh.h"

class Rigidbody;

class Model
{
  public:
	static std::unique_ptr<Model> create(std::string filename, DeviceManager *deviceManager, VkCommandPool commandPool);
	static std::unique_ptr<Model> createBox(DeviceManager *deviceManager, VkCommandPool commandPool,
											std::string diffusePath = "black", std::string specularPath = "black");
	static std::unique_ptr<Model> createSphere(DeviceManager *deviceManager, VkCommandPool commandPool,
											   std::string diffusePath = "black", std::string specularPath = "black");

	void clear();
	void recordDrawCommand(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, uint32_t currentFrame);
	void updateUniformBuffer(UniformBufferObject &ubo, uint32_t currentFrame);
	void createDescriptorSets(VkDevice device, VkDescriptorPool descriptorPool,
							  VkDescriptorSetLayout descriptorSetLayout);
	uint32_t getSize();
	Type getShapeType() const;

  private:
	void load(std::string filename, DeviceManager *deviceManager, VkCommandPool commandPool);
	void createBoxMesh(DeviceManager *deviceManager, VkCommandPool commandPool, std::string diffusePath,
					   std::string specularPath);
	void createSphereMesh(DeviceManager *deviceManager, VkCommandPool commandPool, std::string diffusePath,
						  std::string specularPath);
	void processNode(DeviceManager *deviceManager, VkCommandPool commandPool, aiNode *node, const aiScene *scene);
	void processMesh(DeviceManager *deviceManager, VkCommandPool commandPool, aiMesh *mesh, const aiScene *scene);
	void setShapeType(Type type);

	std::vector<std::unique_ptr<Mesh>> meshes;
	std::vector<std::unique_ptr<Material>> materials;
	Rigidbody *body;
	Type type;
	uint32_t size;
};

#endif