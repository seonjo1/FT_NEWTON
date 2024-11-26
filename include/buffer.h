#ifndef BUFFER_H
# define BUFFER_H

# include "common.h"
# include "deviceManager.h"
# include "vertex.h"
# include <chrono>

struct Vertex;

class Buffer
{
public:
	virtual ~Buffer() = default;
	void clear();
	VkBuffer getBuffer();
	VkDeviceMemory getBufferMemory();
	std::vector<VkBuffer>& getBuffers();

protected:
	Buffer() = default;
	void createBuffer(VkPhysicalDevice physicalDevice, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory);
	void copyBuffer(VkQueue graphicsQueue, VkCommandPool commandPool, VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
	void endSingleTimeCommands(VkQueue graphicsQueue, VkCommandPool commandPool, VkCommandBuffer commandBuffer);
	uint32_t findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties);
	VkCommandBuffer beginSingleTimeCommands(VkCommandPool commandPool);

	VkDevice device;
	std::vector<VkBuffer> buffers;
	std::vector<VkDeviceMemory> buffersMemory;
	uint32_t size;
};


class VertexBuffer : public Buffer
{
public:
	static std::unique_ptr<VertexBuffer> create(std::vector<Vertex> vertices, DeviceManager* deviceManager, VkCommandPool commandPool);
	virtual ~VertexBuffer() = default;
	uint32_t getVerticesSize();

private:
	VertexBuffer() = default;
	void init(std::vector<Vertex> vertices, DeviceManager* deviceManager, VkCommandPool commandPool);
	uint32_t vSize;
};

class IndexBuffer : public Buffer
{
public:
	static std::unique_ptr<IndexBuffer> create(std::vector<uint32_t> indices, DeviceManager* deviceManager, VkCommandPool commandPool);
	virtual ~IndexBuffer() = default;
	uint32_t getIndicesSize();
	
private:
	IndexBuffer() = default;
	void init(std::vector<uint32_t> indices, DeviceManager* deviceManager, VkCommandPool commandPool);
	uint32_t iSize;
};


class UniformBuffer : public Buffer
{
public:
	static std::unique_ptr<UniformBuffer> create(DeviceManager* deviceManager, VkDeviceSize bufferSize);
	void update(VkExtent2D swapChainExtent, uint32_t currentFrame);
	virtual ~UniformBuffer() = default;

private:
	UniformBuffer() = default;
	void init(DeviceManager* deviceManager, VkDeviceSize bufferSize);

	std::vector<void*> buffersMapped;
};

class StagingBuffer : public Buffer
{
public:
	virtual ~StagingBuffer() = default;
	static std::unique_ptr<StagingBuffer> create(DeviceManager* deviceManager, VkDeviceSize imageSize);
	
private:
	StagingBuffer() = default;
	void init(DeviceManager* deviceManager, VkDeviceSize imageSize);
};

#endif