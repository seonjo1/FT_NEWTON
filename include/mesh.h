#ifndef MESH_H
#define MESH_H

#include "buffer.h"
#include "image.h"
#include "physics/BoxShape.h"
#include "physics/SphereShape.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <filesystem>
#include <optional>

class Material;

class Mesh
{
  public:
	~Mesh() = default;
	void clear();
	static std::unique_ptr<Mesh> create(DeviceManager *deviceManager, VkCommandPool commandPool,
										const std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices);
	static std::unique_ptr<Mesh> createBox(DeviceManager *deviceManager, VkCommandPool commandPool,
										   ale::BoxShape *shape, const ale::Transform &xf);
	static std::unique_ptr<Mesh> createSphere(DeviceManager *deviceManager, VkCommandPool commandPool,
											  ale::SphereShape *shape, const ale::Transform &xf);
	static std::unique_ptr<Mesh> createGround(DeviceManager *deviceManager, VkCommandPool commandPool,
											  ale::BoxShape *shape, const ale::Transform &xf);
	void setMaterial(Material *material);
	void createDescriptorSets(VkDevice device, VkDescriptorPool descriptorPool,
							  VkDescriptorSetLayout descriptorSetLayout);
	void recordDrawCommand(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, uint32_t currentFrame);
	VertexBuffer *getVertexBuffer();
	IndexBuffer *getIndexBuffer();
	UniformBuffer *getUniformBuffer();
	VkDescriptorSet *getDescriptor(uint32_t currentFrame);

  private:
	Mesh() = default;
	void init(DeviceManager *deviceManager, VkCommandPool commandPool, const std::vector<Vertex> &vertices,
			  const std::vector<uint32_t> &indices);

	std::unique_ptr<VertexBuffer> vertexBuffer;
	std::unique_ptr<IndexBuffer> indexBuffer;
	std::unique_ptr<UniformBuffer> uniformBuffer;
	Material *material;
	std::vector<VkDescriptorSet> descriptorSets;
	uint32_t primitiveType{GL_TRIANGLES};
};

class Material
{
  public:
	static std::unique_ptr<Material> create(aiMaterial *materialInfo, DeviceManager *deviceManager,
											VkCommandPool commandPool, std::string &dirname);
	static std::unique_ptr<Material> create(DeviceManager *deviceManager, VkCommandPool commandPool,
											std::string diffusePath, std::string specularPath);

	void clear();
	std::unique_ptr<TextureImage> diffuse;
	std::unique_ptr<TextureImage> specular;
	float shininess{32.0f};

  private:
	Material() = default;
	void init(aiMaterial *materialInfo, DeviceManager *deviceManager, VkCommandPool commandPool, std::string &dirname);
	void init(DeviceManager *deviceManager, VkCommandPool commandPool, std::string diffusePath,
			  std::string specularPath);
	std::optional<std::string> getFilePath(aiMaterial *materialInfo, aiTextureType type, std::string &dirname);
};

#endif
