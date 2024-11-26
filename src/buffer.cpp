# include "../include/buffer.h"

std::unique_ptr<VertexBuffer> VertexBuffer::create(std::vector<Vertex> vertices, DeviceManager* deviceManager, VkCommandPool commandPool)
{
	std::unique_ptr<VertexBuffer> vertexBuffer(new VertexBuffer());
	vertexBuffer->init(vertices, deviceManager, commandPool);
	return vertexBuffer;
}

/*
	[버텍스 버퍼 생성]
	1. 스테이징 버퍼 생성
	2. 스테이징 버퍼에 정점 정보 복사
	3. 버텍스 버퍼 생성
	4. 스테이징 버퍼의 정점 정보를 버텍스 버퍼에 복사
	5. 스테이징 버퍼 리소스 해제
*/ 
void VertexBuffer::init(std::vector<Vertex> vertices, DeviceManager* deviceManager, VkCommandPool commandPool)
{
	device = deviceManager->getLogicalDevice();
	VkPhysicalDevice physicalDevice	= deviceManager->getPhysicalDevice();
	
	vSize = vertices.size();
	size = 1;
	buffers.resize(size);
	buffersMemory.resize(size);

	// 정점 정보 크기		
	VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

	// 스테이징 버퍼 객체, 스테이징 버퍼 메모리 객체 생성
	VkBuffer stagingBuffer;
	VkDeviceMemory stagingBufferMemory;

	// [스테이징 버퍼 생성]
	// 용도)
	//		VK_BUFFER_USAGE_TRANSFER_SRC_BIT : 데이터 전송의 소스로 사용
	// 속성)
	// 		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT  : CPU에서 GPU 메모리에 접근이 가능한 설정
	// 		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT : CPU에서 GPU 메모리의 값을 수정하면 그 즉시 GPU 메모리와 캐시에 해당 값을 수정하는 설정 
	//      									  (원래는 CPU에서 GPU 메모리 값을 수정하면 GPU 캐시를 플러쉬하여 다시 캐시에 값을 올리는 형식으로 동작)
	createBuffer(physicalDevice, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

	// [스테이징 버퍼(GPU 메모리)에 정점 정보 입력]
	void* data; // GPU 메모리에 매핑될 CPU 메모리 가상 포인터
	vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data); 	// GPU 메모리에 매핑된 CPU 메모리 포인터 반환 (현재는 버텍스 버퍼 메모리 전부를 매핑)
	memcpy(data, vertices.data(), (size_t) bufferSize);					// CPU 메모리 포인터는 가상포인터로 data에 정점 정보를 복사하면 GPU에 즉시 반영
																		// 실제 CPU 메모리에 저장되는게 아닌 CPU 메모리 포인터는 가상 포인터역할만 하고 GPU 메모리에 바로 저장
																		// 메모리 유형의 속성에 의해 GPU 캐시를 플러쉬할 필요없이 즉시 적용
																		// VK_MEMORY_PROPERTY_HOST_COHERENT_BIT 속성 덕분
																		// 만약 해당 속성 없이 플러쉬도 안 하면 GPU에서 변경사항이 바로 적용되지 않음
	vkUnmapMemory(device, stagingBufferMemory);							// GPU 메모리 매핑 해제


	// [버텍스 버퍼 생성]
	// 용도)
	//		VK_BUFFER_USAGE_TRANSFER_DST_BIT : 데이터 전송의 대상으로 사용
	// 속성)
	// 		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT : 버퍼를 정점 데이터를 저장하고 처리하는 용도로 설정.
	// 		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT : GPU 전용 메모리에 데이터를 저장하여, GPU가 최적화된 방식으로 접근할 수 있게 함.
	createBuffer(physicalDevice, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffers[0], buffersMemory[0]);

	// [스테이징 버퍼에서 버텍스 버퍼로 메모리 이동]
	copyBuffer(deviceManager->getGraphicsQueue(), commandPool, stagingBuffer, buffers[0], bufferSize);

	// 스테이징 버퍼와 할당된 메모리 해제
	vkDestroyBuffer(device, stagingBuffer, nullptr);
	vkFreeMemory(device, stagingBufferMemory, nullptr);	
}

uint32_t VertexBuffer::getVerticesSize()
{
	return vSize;
}

std::unique_ptr<IndexBuffer> IndexBuffer::create(std::vector<uint32_t> indices, DeviceManager* deviceManager, VkCommandPool commandPool)
{
	std::unique_ptr<IndexBuffer> indexBuffer(new IndexBuffer());
	indexBuffer->init(indices, deviceManager, commandPool);
	return indexBuffer;
}

void IndexBuffer::init(std::vector<uint32_t> indices, DeviceManager* deviceManager, VkCommandPool commandPool)
{
	device = deviceManager->getLogicalDevice();
	VkPhysicalDevice physicalDevice	= deviceManager->getPhysicalDevice();

	iSize = indices.size();
	size = 1;
	buffers.resize(size);
	buffersMemory.resize(size);

	VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();

	VkBuffer stagingBuffer;
	VkDeviceMemory stagingBufferMemory;
	createBuffer(physicalDevice, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

	void* data;
	vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
	memcpy(data, indices.data(), (size_t) bufferSize);
	vkUnmapMemory(device, stagingBufferMemory);

	// [버텍스 버퍼 생성]
	// 속성)
	// 		VK_BUFFER_USAGE_INDEX_BUFFER_BIT : 버퍼를 인덱스 데이터를 저장하고 처리하는 용도로 설정.
	// 		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT : GPU 전용 메모리에 데이터를 저장하여, GPU가 최적화된 방식으로 접근할 수 있게 함.
	createBuffer(physicalDevice, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffers[0], buffersMemory[0]);

	copyBuffer(deviceManager->getGraphicsQueue(), commandPool, stagingBuffer, buffers[0], bufferSize);

	vkDestroyBuffer(device, stagingBuffer, nullptr);
	vkFreeMemory(device, stagingBufferMemory, nullptr);
}

uint32_t IndexBuffer::getIndicesSize()
{
	return iSize;
}

std::unique_ptr<UniformBuffer> UniformBuffer::create(DeviceManager* deviceManager, VkDeviceSize bufferSize)
{
	std::unique_ptr<UniformBuffer> uniformBuffer(new UniformBuffer());
	uniformBuffer->init(deviceManager, bufferSize);
	return uniformBuffer;
}

void UniformBuffer::init(DeviceManager* deviceManager, VkDeviceSize bufferSize)
{
	device = deviceManager->getLogicalDevice();
	VkPhysicalDevice physicalDevice	= deviceManager->getPhysicalDevice();

	size = MAX_FRAMES_IN_FLIGHT;

	// 각 요소들을 동시에 처리 가능한 최대 프레임 수만큼 만들어 둔다.
	buffers.resize(size);		// 유니폼 버퍼 객체
	buffersMemory.resize(size);	// 유니폼 버퍼에 할당할 메모리
	buffersMapped.resize(size);	// GPU 메모리에 매핑할 CPU 메모리 포인터

	for (size_t i = 0; i < size; i++) {
		// 유니폼 버퍼 객체 생성 + 메모리 할당 + 바인딩
		createBuffer(physicalDevice, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, buffers[i], buffersMemory[i]);
		// GPU 메모리 CPU 가상 포인터에 매핑
		vkMapMemory(device, buffersMemory[i], 0, bufferSize, 0, &buffersMapped[i]);
	}
}

void UniformBuffer::update(VkExtent2D swapChainExtent, uint32_t currentImage)
{
	static auto startTime = std::chrono::high_resolution_clock::now();

	auto currentTime = std::chrono::high_resolution_clock::now();
	float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

	// 이번 프레임의 유니폼 변수 값 구하기
	// 1초에 90도씩 회전하는 model view projection 변환 생성
	UniformBufferObject ubo{};
	ubo.model = glm::rotate(glm::mat4(1.0f), time * glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	ubo.view = glm::lookAt(glm::vec3(2.0f, 2.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	ubo.proj = glm::perspective(glm::radians(45.0f), swapChainExtent.width / (float) swapChainExtent.height, 0.1f, 10.0f);
	ubo.proj[1][1] *= -1;

	// 유니폼 변수를 매핑된 GPU 메모리에 복사
	memcpy(buffersMapped[currentImage], &ubo, sizeof(ubo));
}

std::unique_ptr<StagingBuffer> StagingBuffer::create(DeviceManager* deviceManager, VkDeviceSize imageSize)
{
	std::unique_ptr<StagingBuffer> stagingBuffer(new StagingBuffer());
	stagingBuffer->init(deviceManager, imageSize);
	return stagingBuffer;
}

void StagingBuffer::init(DeviceManager* deviceManager, VkDeviceSize imageSize)
{
	device = deviceManager->getLogicalDevice();
	size = 1;
	buffers.resize(size);
	buffersMemory.resize(size);
	createBuffer(deviceManager->getPhysicalDevice(), imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, buffers[0], buffersMemory[0]);
}

/*
	[버퍼 생성]
	1. 버퍼 객체 생성
	2. 버퍼 메모리 할당
	3. 버퍼 객체에 할당한 메모리 바인딩
*/ 
void Buffer::createBuffer(VkPhysicalDevice physicalDevice, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory) {
	// 버퍼 객체를 생성하기 위한 구조체 (GPU 메모리에 데이터 저장 공간을 할당하는 데 필요한 설정을 정의)
	VkBufferCreateInfo bufferInfo{};
	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.size = size;										// 버퍼의 크기 지정
	bufferInfo.usage = usage;									// 버퍼의 용도 지정
	bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;			// 버퍼를 하나의 큐 패밀리에서 쓸지, 여러 큐 패밀리에서 공유할지 설정 (현재 단일 큐 패밀리에서 사용하도록 설정)
																// 여러 큐 패밀리에서 공유하는 모드 사용시 추가 설정 필요
	// [버퍼 생성]
	// 버퍼를 생성하지만 할당은 안되어있는 상태로 만들어짐       
	if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
		throw std::runtime_error("failed to create buffer!");
	}

	// [버퍼에 메모리 할당]
	// 메모리 할당 요구사항 조회
	VkMemoryRequirements memRequirements;
	vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

	// 메모리 할당을 위한 구조체
	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;	// 할당할 메모리 크기
	// 메모리 요구사항 설정 (GPU 메모리 유형 중 buffer와 호환되고 properties 속성들과 일치하는 것 찾아 저장)
	// 메모리 유형 - GPU 메모리는 구역마다 유형이 다르다. (memoryTypeBits는 buffer가 호환되는 GPU의 메모리 유형이 전부 담겨있음)
	// 메모리 유형의 속성 - 메모리 유형마다 특성을 가지고 있음
	allocInfo.memoryTypeIndex = findMemoryType(physicalDevice, memRequirements.memoryTypeBits, properties);

	// 버퍼 메모리 할당
	if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
		throw std::runtime_error("failed to allocate buffer memory!");
	}

	// 버퍼 객체에 할당된 메모리를 바인딩 (4번째 매개변수는 할당할 메모리의 offset)
	vkBindBufferMemory(device, buffer, bufferMemory, 0);
}


/*
	GPU와 buffer가 호환되는 메모리 유형중 properties에 해당하는 속성들을 갖는 메모리 유형 찾기
*/
uint32_t Buffer::findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) {
	// GPU에서 사용한 메모리 유형을 가져온다.
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		// typeFilter & (1 << i) : GPU의 메모리 유형중 버퍼와 호환되는 것인지 판단
		// memProperties.memoryTypes[i].propertyFlags & properties : GPU 메모리 유형의 속성이 properties와 일치하는지 판단
		if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
			// 해당 메모리 유형 반환
			return i;
		}
	}

	throw std::runtime_error("failed to find suitable memory type!");
}


// srcBuffer 에서 dstBuffer 로 데이터 복사
void Buffer::copyBuffer(VkQueue graphicsQueue, VkCommandPool commandPool, VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size) {
	VkCommandBuffer commandBuffer = beginSingleTimeCommands(commandPool);

	VkBufferCopy copyRegion{}; 	// 복사할 버퍼 영역을 지정 (크기, src 와 dst의 시작 offset 등)
	copyRegion.size = size;		// 복사할 버퍼 크기 설정
	vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion); // 커맨드 버퍼에 복사 명령 기록

	endSingleTimeCommands(graphicsQueue, commandPool, commandBuffer);		
}


// 한 번만 실행할 커맨드 버퍼 생성 및 기록 시작
VkCommandBuffer Buffer::beginSingleTimeCommands(VkCommandPool commandPool) {
	// 커맨드 버퍼 할당을 위한 구조체 
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;		// PRIMARY LEVEL 등록 (해당 커맨드 버퍼가 큐에 단독으로 제출될 수 있음)
	allocInfo.commandPool = commandPool;					// 커맨드 풀 지정
	allocInfo.commandBufferCount = 1;						// 커맨드 버퍼 개수 지정

	// 커맨드 버퍼 생성
	VkCommandBuffer commandBuffer;
	vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

	// 커맨드 버퍼 기록을 위한 정보 객체
	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;  // 커맨드 버퍼를 1번만 제출

	// GPU에 필요한 작업을 모두 커맨드 버퍼에 기록하기 시작
	vkBeginCommandBuffer(commandBuffer, &beginInfo);

	return commandBuffer;
}


// 한 번만 실행할 커맨드 버퍼 기록 중지 및 큐에 커맨드 버퍼 제출
void Buffer::endSingleTimeCommands(VkQueue graphicsQueue, VkCommandPool commandPool, VkCommandBuffer commandBuffer) {
	// 커맨드 버퍼 기록 중지
	vkEndCommandBuffer(commandBuffer);

	// 복사 커맨드 버퍼 제출 정보 객체 생성
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;								// 커맨드 버퍼 개수
	submitInfo.pCommandBuffers = &commandBuffer;					// 커맨드 버퍼 등록

	vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);	// 커맨드 버퍼 큐에 제출
	vkQueueWaitIdle(graphicsQueue);									// 그래픽스 큐 작업 종료 대기

	vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);	// 커맨드 버퍼 제거
}

void Buffer::clear()
{
	for (size_t i = 0; i < size; i++) {
		vkDestroyBuffer(device, buffers[i], nullptr);	// 유니폼 버퍼 객체 삭제
		vkFreeMemory(device, buffersMemory[i], nullptr);	// 유니폼 버퍼에 할당된 메모리 삭제
	}
}

VkBuffer Buffer::getBuffer()
{
	return buffers[0];
}

VkDeviceMemory Buffer::getBufferMemory()
{
	return buffersMemory[0];
}

std::vector<VkBuffer>& Buffer::getBuffers()
{
	return buffers;
}
