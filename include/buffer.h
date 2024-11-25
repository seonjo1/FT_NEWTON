#ifndef BUFFER_H
# define BUFFER_H

# include "common.h"
# include "deviceManager.h"

# include <chrono>

struct Vertex;

class Buffer
{
public:
	virtual ~Buffer() = default;
	void clear();
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
	virtual ~VertexBuffer() = default;
	static std::unique_ptr<VertexBuffer> create(std::vector<Vertex> vertices, DeviceManager* deviceManager, VkCommandPool commandPool);
	
private:
	VertexBuffer() = default;
	void init(std::vector<Vertex> vertices, DeviceManager* deviceManager, VkCommandPool commandPool);

};

class IndexBuffer : public Buffer
{
public:
	virtual ~IndexBuffer() = default;
	static std::unique_ptr<IndexBuffer> create(std::vector<uint32_t> indices, DeviceManager* deviceManager, VkCommandPool commandPool);
	
private:
	IndexBuffer() = default;
	void init(std::vector<uint32_t> indices, DeviceManager* deviceManager, VkCommandPool commandPool);

};


class UniformBuffer : public Buffer
{
public:
	void update(VkExtent2D swapChainExtent, uint32_t currentImage);
	virtual ~UniformBuffer() = default;
	static std::unique_ptr<UniformBuffer> create(DeviceManager* deviceManager);

private:
	UniformBuffer() = default;
	void init(DeviceManager* deviceManager);

	std::vector<void*> buffersMapped;
};


struct Vertex {
	glm::vec3 pos;
	glm::vec3 color;
	glm::vec2 texCoord;

	// 정점 데이터가 전달되는 방법을 알려주는 구조체 반환하는 함수
	static VkVertexInputBindingDescription getBindingDescription() {
		// 파이프라인에 정점 데이터가 전달되는 방법을 알려주는 구조체
		VkVertexInputBindingDescription bindingDescription{};		
		bindingDescription.binding = 0;								// 버텍스 바인딩 포인트 (현재 0번에 vertex 정보 바인딩)
		bindingDescription.stride = sizeof(Vertex);					// 버텍스 1개 단위의 정보 크기
		bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX; // 정점 데이터 처리 방법
																	// 1. VK_VERTEX_INPUT_RATE_VERTEX : 정점별로 데이터 처리
																	// 2. VK_VERTEX_INPUT_RATE_INSTANCE : 인스턴스별로 데이터 처리
		return bindingDescription;
	}

	// 정점 속성별 데이터 형식과 위치를 지정하는 구조체 반환하는 함수
	static std::array<VkVertexInputAttributeDescription, 3> getAttributeDescriptions() {
		// 정점 속성의 데이터 형식과 위치를 지정하는 구조체
		std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions{};

		// pos 속성 정보 입력
		attributeDescriptions[0].binding = 0;							// 버텍스 버퍼의 바인딩 포인트
		attributeDescriptions[0].location = 0;							// 버텍스 셰이더의 어떤 location에 대응되는지 지정
		attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;	// 저장되는 데이터 형식 (VK_FORMAT_R32G32B32_SFLOAT = vec3)
		attributeDescriptions[0].offset = offsetof(Vertex, pos);		// 버텍스 구조체에서 해당 속성이 시작되는 위치

		// color 속성 정보 입력
		attributeDescriptions[1].binding = 0;
		attributeDescriptions[1].location = 1;
		attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[1].offset = offsetof(Vertex, color);

		// texCoord 속성 정보 입력
		attributeDescriptions[2].binding = 0;
		attributeDescriptions[2].location = 2;
		attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
		attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

		return attributeDescriptions;
	}
};

struct UniformBufferObject {
	alignas(16) glm::mat4 model;
	alignas(16) glm::mat4 view;
	alignas(16) glm::mat4 proj;
};

#endif