#include "../include/image.h"
#include "../include/buffer.h"

# define STB_IMAGE_IMPLEMENTATION
# include <stb/stb_image.h>

void Image::clear()
{
	vkDestroyImageView(device, imageView, nullptr);
	vkDestroyImage(device, image, nullptr);
	vkFreeMemory(device, imageMemory, nullptr);
}

VkImageView Image::createSwapChainImageView(VkDevice device, VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels)
{
	// 이미지 뷰 정보 생성
	VkImageViewCreateInfo viewInfo{};
	viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	viewInfo.image = image;													// 이미지 핸들
	viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;								// 이미지 타입
	viewInfo.format = format;												// 이미지 포맷
	viewInfo.subresourceRange.aspectMask = aspectFlags;  					// 이미지 형식 결정 (color / depth / stencil 등)
	viewInfo.subresourceRange.baseMipLevel = 0;                          	// 렌더링할 mipmap 단계 설정
	viewInfo.subresourceRange.levelCount = mipLevels;                            	// baseMipLevel 기준으로 몇 개의 MipLevel을 더 사용할지 설정 (실제 mipmap 만드는 건 따로 해줘야함)
	viewInfo.subresourceRange.baseArrayLayer = 0;                        	// ImageView가 참조하는 이미지 레이어의 시작 위치 정의
	viewInfo.subresourceRange.layerCount = 1;                            	// 스왑 체인에서 설정한 이미지 레이어 개수

	// 이미지 뷰 생성
	VkImageView imageView;
	if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
		throw std::runtime_error("failed to create image view!");
	}

	return imageView;
}

/*
	[이미지 객체 생성 및 메모리 할당]
	1. 이미지 객체 생성
	2. 이미지 객체가 사용할 메모리 할당
	3. 이미지 객체에 할당한 메모리 바인딩
*/
void Image::createImage(DeviceManager* deviceManager, uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties) {

	VkPhysicalDevice physicalDevice = deviceManager->getPhysicalDevice();

	// 이미지 객체를 만드는데 사용되는 구조체
	VkImageCreateInfo imageInfo{};
	imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageInfo.imageType = VK_IMAGE_TYPE_2D;					// 이미지의 차원을 설정
	imageInfo.extent.width = width;							// 이미지의 너비 지정
	imageInfo.extent.height = height;						// 이미지의 높이 지정 
	imageInfo.extent.depth = 1;								// 이미지의 깊이 지정 (2D 이미지의 경우 depth는 1로 지정해야 함)
	imageInfo.mipLevels = mipLevels;						// 생성할 mipLevel의 개수 지정
	imageInfo.arrayLayers = 1;								// 생성할 이미지 레이어 수 (큐브맵의 경우 6개 생성)
	imageInfo.format = imageFormat;								// 이미지의 포맷을 지정하며, 채널 구성과 각 채널의 비트 수를 정의
	imageInfo.tiling = tiling;								// 이미지를 GPU 메모리에 배치할 때 메모리 레이아웃을 결정하는 설정 (CPU에서도 접근 가능하게 할꺼냐, GPU에만 접근 가능하게 최적화 할거냐 결정) 
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;	// 이미지 초기 레이아웃 설정 (이미지가 메모리에 배치될 때 초기 상태를 정의)
	imageInfo.usage = usage;								// 이미지의 사용 용도 결정
	imageInfo.samples = numSamples;							// 멀티 샘플링을 위한 샘플 개수
	imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;		// 이미지의 큐 공유 모드 설정 (VK_SHARING_MODE_EXCLUSIVE: 한 번에 하나의 큐 패밀리에서만 접근 가능한 단일 큐 모드)

	// 이미지 객체 생성
	if (vkCreateImage(device, &imageInfo, nullptr, &image) != VK_SUCCESS) {
		throw std::runtime_error("failed to create image!");
	}

	// 이미지에 필요한 메모리 요구 사항을 조회 
	VkMemoryRequirements memRequirements;
	vkGetImageMemoryRequirements(device, image, &memRequirements);

	// 메모리 할당을 위한 구조체
	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;											// 메모리 크기 설정
	allocInfo.memoryTypeIndex = findMemoryType(physicalDevice, memRequirements.memoryTypeBits, properties);		// 메모리 유형과 속성 설정

	// 이미지를 위한 메모리 할당
	if (vkAllocateMemory(device, &allocInfo, nullptr, &imageMemory) != VK_SUCCESS) {
		throw std::runtime_error("failed to allocate image memory!");
	}

	// 이미지에 할당한 메모리 바인딩
	vkBindImageMemory(device, image, imageMemory, 0);
}


/*
	GPU와 buffer가 호환되는 메모리 유형중 properties에 해당하는 속성들을 갖는 메모리 유형 찾기
*/
uint32_t Image::findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) {
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

// 이미지 뷰 생성
void Image::createImageView(VkImageAspectFlags aspectFlags, uint32_t mipLevels) {
	// 이미지 뷰 정보 생성
	VkImageViewCreateInfo viewInfo{};
	viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	viewInfo.image = image;													// 이미지 핸들
	viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;								// 이미지 타입
	viewInfo.format = imageFormat;												// 이미지 포맷
	viewInfo.subresourceRange.aspectMask = aspectFlags;  					// 이미지 형식 결정 (color / depth / stencil 등)
	viewInfo.subresourceRange.baseMipLevel = 0;                          	// 렌더링할 mipmap 단계 설정
	viewInfo.subresourceRange.levelCount = mipLevels;                            	// baseMipLevel 기준으로 몇 개의 MipLevel을 더 사용할지 설정 (실제 mipmap 만드는 건 따로 해줘야함)
	viewInfo.subresourceRange.baseArrayLayer = 0;                        	// ImageView가 참조하는 이미지 레이어의 시작 위치 정의
	viewInfo.subresourceRange.layerCount = 1;                            	// 스왑 체인에서 설정한 이미지 레이어 개수

	if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
		throw std::runtime_error("failed to create image view!");
	}
}

VkImageView Image::getImageView()
{
	return imageView;
}

std::unique_ptr<ColorImage> ColorImage::create(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent)
{
	std::unique_ptr<ColorImage> colorImage(new ColorImage());
	colorImage->init(deviceManager, format, swapChainExtent);
	return colorImage;

}

// 멀티샘플링용 color Image생성
void ColorImage::init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent) {
	device = deviceManager->getLogicalDevice();
	imageFormat = format;
	createImage(deviceManager, swapChainExtent.width, swapChainExtent.height, 1, deviceManager->getMsaaSamples(), VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	createImageView(VK_IMAGE_ASPECT_COLOR_BIT, 1);
}

std::unique_ptr<DepthImage> DepthImage::create(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent)
{
	std::unique_ptr<DepthImage> depthImage(new DepthImage());
	depthImage->init(deviceManager, format, swapChainExtent);
	return depthImage;
}

// Depth test에 쓰일 image, imageView 준비
void DepthImage::init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent) {
	device = deviceManager->getLogicalDevice();
	imageFormat = format;
	// depth image의 format 결정
	createImage(deviceManager, swapChainExtent.width, swapChainExtent.height, 1, deviceManager->getMsaaSamples(), VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	createImageView(VK_IMAGE_ASPECT_DEPTH_BIT, 1);
}



std::unique_ptr<TextureImage> TextureImage::create(const char* path, DeviceManager* deviceManager, VkCommandPool commandPool)
{
	std::unique_ptr<TextureImage> textureImage(new TextureImage());
	textureImage->init(path, deviceManager, commandPool);
	return textureImage;
}

std::unique_ptr<TextureImage> TextureImage::createBlackTexture(DeviceManager* deviceManager, VkCommandPool commandPool)
{
	std::unique_ptr<TextureImage> blackTextureImage(new TextureImage());
	blackTextureImage->init(deviceManager, commandPool);
	return blackTextureImage;
}

void TextureImage::init(const char* path, DeviceManager* deviceManager, VkCommandPool commandPool)
{
	device = deviceManager->getLogicalDevice();
	createTextureImage(path, deviceManager, commandPool);
	createTextureImageView();
	createTextureSampler(deviceManager->getPhysicalDevice());
}

void TextureImage::init(DeviceManager* deviceManager, VkCommandPool commandPool)
{
	device = deviceManager->getLogicalDevice();
	createBlackTextureImage(deviceManager, commandPool);
	createTextureImageView();
	createTextureSampler(deviceManager->getPhysicalDevice());
}

void TextureImage::createBlackTextureImage(DeviceManager* deviceManager, VkCommandPool commandPool) {
	VkQueue graphicsQueue = deviceManager->getGraphicsQueue();
	imageFormat = VK_FORMAT_R8G8B8A8_SRGB;


	int texWidth = 256;  // 텍스처 너비
	int texHeight = 256; // 텍스처 높이
	VkDeviceSize imageSize = texWidth * texHeight * 4;
	mipLevels = 1;
	std::vector<uint8_t> blackImage(imageSize, 0);

	// 스테이징 버퍼 생성
	std::unique_ptr<StagingBuffer> stagingBuffer = StagingBuffer::create(deviceManager, imageSize);

	// 스테이징 버퍼에 이미지 데이터 복사
	void* data;
	vkMapMemory(device, stagingBuffer->getBufferMemory(), 0, imageSize, 0, &data);
	memcpy(data, blackImage.data(), static_cast<size_t>(imageSize));
	vkUnmapMemory(device, stagingBuffer->getBufferMemory());

	// 이미지 객체 생성
	createImage(deviceManager, texWidth, texHeight, mipLevels, VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	// Top stage 끝나고 베리어를 이용한 이미지 전환 설정 
	// (같은 작업 큐에서 Transfer 단계 들어가는 다른 작업들 해당 베리어 작업이 끝날때까지 stop)
	transitionImageLayout(graphicsQueue, commandPool, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, mipLevels);
	
	// 커맨드 버퍼를 이용한 버퍼 -> 이미지 데이터 복사 실행
	copyBufferToImage(graphicsQueue, commandPool, stagingBuffer->getBuffer(), static_cast<uint32_t>(texWidth), static_cast<uint32_t>(texHeight));

	// Transfer 끝나고 베리어를 이용한 이미지 전환 설정
	// (같은 작업 큐에서 Fragment shader 단계 들어가는 다른 작업들 해당 베리어 작업이 끝날때까지 stop)
	// mipmap 생성에서 하는걸로 변경
	// transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, mipLevels);

	// 스테이징 버퍼 삭제
	stagingBuffer->clear();

	// mipmap 생성
	generateMipmaps(deviceManager, commandPool, texWidth, texHeight, mipLevels);
}

void TextureImage::createTextureImage(const char* path, DeviceManager* deviceManager, VkCommandPool commandPool) {
	VkQueue graphicsQueue = deviceManager->getGraphicsQueue();
	imageFormat = VK_FORMAT_R8G8B8A8_SRGB;

	int texWidth, texHeight, texChannels;
	stbi_uc* pixels = stbi_load(path, &texWidth, &texHeight, &texChannels, STBI_rgb_alpha); // 알파 채널을 포함하여 rgba 픽셀로 이미지 저장
	VkDeviceSize imageSize = texWidth * texHeight * 4;  // 이미지 크기 (픽셀당 4byte)
	mipLevels = static_cast<uint32_t>(std::floor(std::log2(std::max(texWidth, texHeight)))) + 1; // mipLevel 설정


	if (!pixels) {
		// 로드 실패시 오류 처리
		throw std::runtime_error("failed to load texture image!");
	}

	// 스테이징 버퍼 생성
	std::unique_ptr<StagingBuffer> stagingBuffer = StagingBuffer::create(deviceManager, imageSize);

	// 스테이징 버퍼에 이미지 데이터 복사
	void* data;
	vkMapMemory(device, stagingBuffer->getBufferMemory(), 0, imageSize, 0, &data);
	memcpy(data, pixels, static_cast<size_t>(imageSize));
	vkUnmapMemory(device, stagingBuffer->getBufferMemory());

	// 이미지 데이터 해제
	stbi_image_free(pixels);

	// 이미지 객체 생성
	createImage(deviceManager, texWidth, texHeight, mipLevels, VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	// Top stage 끝나고 베리어를 이용한 이미지 전환 설정 
	// (같은 작업 큐에서 Transfer 단계 들어가는 다른 작업들 해당 베리어 작업이 끝날때까지 stop)
	transitionImageLayout(graphicsQueue, commandPool, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, mipLevels);
	
	// 커맨드 버퍼를 이용한 버퍼 -> 이미지 데이터 복사 실행
	copyBufferToImage(graphicsQueue, commandPool, stagingBuffer->getBuffer(), static_cast<uint32_t>(texWidth), static_cast<uint32_t>(texHeight));

	// Transfer 끝나고 베리어를 이용한 이미지 전환 설정
	// (같은 작업 큐에서 Fragment shader 단계 들어가는 다른 작업들 해당 베리어 작업이 끝날때까지 stop)
	// mipmap 생성에서 하는걸로 변경
	// transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, mipLevels);

	// 스테이징 버퍼 삭제
	stagingBuffer->clear();

	// mipmap 생성
	generateMipmaps(deviceManager, commandPool, texWidth, texHeight, mipLevels);

}

void TextureImage::generateMipmaps(DeviceManager* deviceManager, VkCommandPool commandPool, int32_t texWidth, int32_t texHeight, uint32_t mipLevels) {
	VkQueue graphicsQueue = deviceManager->getGraphicsQueue();
	VkPhysicalDevice physicalDevice = deviceManager->getPhysicalDevice();

	// 이미지 포맷이 선형 필터링을 사용한 Blit 작업을 지원하는지 확인
	VkFormatProperties formatProperties;
	vkGetPhysicalDeviceFormatProperties(physicalDevice, imageFormat, &formatProperties);

	if (!(formatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT)) {
		throw std::runtime_error("texture image format does not support linear blitting!");
	}

	// 커맨드 버퍼 생성 및 기록 시작
	VkCommandBuffer commandBuffer = beginSingleTimeCommands(commandPool);

	// 베리어 생성
	VkImageMemoryBarrier barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	barrier.image = image;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	barrier.subresourceRange.baseArrayLayer = 0;
	barrier.subresourceRange.layerCount = 1;
	barrier.subresourceRange.levelCount = 1;

	int32_t mipWidth = texWidth;
	int32_t mipHeight = texHeight;

	// miplevel 0 ~ mipLevels
	for (uint32_t i = 1; i < mipLevels; i++) {
		barrier.subresourceRange.baseMipLevel = i - 1;								// 해당 mipmap에 대한 barrier 설정
		barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;					// 데이터 쓰기에 적합한 레이아웃
		barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;					// 데이터 읽기에 적합한 레이아웃
		barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;						// 쓰기 권한 on
		barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;						// 읽기 권한 on

		// 파이프라인 베리어 설정 (GPU 특정 작업간의 동기화 설정)
		// 이전 단계의 mipmap 복사가 끝나야, 다음 단계 mipmap 복사가 시작되게 베리어 설정
		vkCmdPipelineBarrier(commandBuffer,
			VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
			0, nullptr,
			0, nullptr,
			1, &barrier);

		// blit 작업시 소스와 대상의 복사 범위를 지정하는 구조체
		VkImageBlit blit{};
		blit.srcOffsets[0] = {0, 0, 0};
		blit.srcOffsets[1] = {mipWidth, mipHeight, 1};														// 소스의 범위 설정
		blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		blit.srcSubresource.mipLevel = i - 1;																// 소스의 mipLevel 설정
		blit.srcSubresource.baseArrayLayer = 0;
		blit.srcSubresource.layerCount = 1;
		blit.dstOffsets[0] = {0, 0, 0};
		blit.dstOffsets[1] = { mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1, 1 };		// 대상의 범위 설정
		blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		blit.dstSubresource.mipLevel = i;																	// 대상의 mipLevel 설정
		blit.dstSubresource.baseArrayLayer = 0;
		blit.dstSubresource.layerCount = 1;

		// 소스 miplevel 을 대상 miplevel로 설정에 맞게 복사
		vkCmdBlitImage(commandBuffer,
			image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
			image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
			1, &blit,
			VK_FILTER_LINEAR); // 선형적으로 복사

		// shader 단계에서 사용하기 전에 Bllit 단계가 끝나기를 기다림
		barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
		barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

		vkCmdPipelineBarrier(commandBuffer,
			VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0,
			0, nullptr,
			0, nullptr,
			1, &barrier);

		if (mipWidth > 1) mipWidth /= 2;
		if (mipHeight > 1) mipHeight /= 2;
	}

	// 마지막 단계 miplevel 처리
	barrier.subresourceRange.baseMipLevel = mipLevels - 1;
	barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
	barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

	vkCmdPipelineBarrier(commandBuffer,
		VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0,
		0, nullptr,
		0, nullptr,
		1, &barrier);

	endSingleTimeCommands(graphicsQueue, commandPool, commandBuffer);
}

// 커맨드 버퍼 제출을 통해 버퍼 -> 이미지 데이터 복사 
void TextureImage::copyBufferToImage(VkQueue graphicsQueue, VkCommandPool commandPool, VkBuffer buffer, uint32_t width, uint32_t height) {
	// 커맨드 버퍼 생성 및 기록 시작
	VkCommandBuffer commandBuffer = beginSingleTimeCommands(commandPool);

	// 버퍼 -> 이미지 복사를 위한 정보
	VkBufferImageCopy region{};
	region.bufferOffset = 0;											// 복사할 버퍼의 시작 위치 offset
	region.bufferRowLength = 0;											// 저장될 공간의 row 당 픽셀 수 (0으로 하면 이미지 너비에 자동으로 맞춰진다.)
	region.bufferImageHeight = 0;										// 저장될 공간의 col 당 픽셀 수 (0으로 하면 이미지 높이에 자동으로 맞춰진다.)
	region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;		// 이미지의 데이터 타입 (현재는 컬러값을 복사)
	region.imageSubresource.mipLevel = 0;								// 이미지의 miplevel 설정
	region.imageSubresource.baseArrayLayer = 0;							// 이미지의 시작 layer 설정 (cubemap과 같은 경우 여러 레이어 존재)
	region.imageSubresource.layerCount = 1;								// 이미지 layer 개수
	region.imageOffset = {0, 0, 0};										// 이미지의 저장할 시작 위치
	region.imageExtent = {												// 이미지의 저장할 너비, 높이, 깊이
		width,
		height,
		1
	};

	// 커맨드 버퍼에 버퍼 -> 이미지로 데이터 복사하는 명령 기록
	vkCmdCopyBufferToImage(commandBuffer, buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

	// 커맨드 버퍼 기록 종료 및 제출
	endSingleTimeCommands(graphicsQueue, commandPool, commandBuffer);
}


// 이미지 레이아웃, 접근 권한을 변경할 수 있는 베리어를 커맨드 버퍼에 기록
void TextureImage::transitionImageLayout(VkQueue graphicsQueue, VkCommandPool commandPool, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels) {
	VkCommandBuffer commandBuffer = beginSingleTimeCommands(commandPool);			// 커맨드 버퍼 생성 및 기록 시작

	// 베리어 생성을 위한 구조체
	VkImageMemoryBarrier barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	barrier.oldLayout = oldLayout;										// src 단계까지의 이미지 레이아웃
	barrier.newLayout = newLayout;										// src 단계 이후 적용시킬 새로운 이미지 레이아웃
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;				// 베리어 전환 적용 후 리소스 소유권을 넘겨줄 src 큐 패밀리 (현재는 동일 큐 패밀리에서 실행)
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;				// 리소스 소유권을 받을 dst 큐 패밀리로 dst 큐패밀리에는 큐 전체에 동기화가 적용 (현재는 동일 큐 패밀리에서 실행)
	barrier.image = image;												// 배리어 적용할 이미지 객체
	barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;	// 전환 작업의 적용 대상을 color bit 으로 설정
	barrier.subresourceRange.baseMipLevel = 0;							// 전환 작업을 시작할 miplevel
	barrier.subresourceRange.levelCount = mipLevels;					// 전환 작업을 적용할 miplevel의 개수
	barrier.subresourceRange.baseArrayLayer = 0;						// 전환 작업을 시작할 레이어 인덱스
	barrier.subresourceRange.layerCount = 1;							// 전환 작업을 적용할 레이어 개수

	VkPipelineStageFlags sourceStage;									// 파이프라인의 sourceStage 단계가 끝나면 배리어 전환 실행 
	VkPipelineStageFlags destinationStage;								// destinationStage 단계는 배리어 전환이 끝날때까지 대기

	if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
		// 이미지 복사 전에 이미지 레이아웃, 접근 권한 변경
		barrier.srcAccessMask = 0;									// 접근 제한 x
		barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;		// 쓰기 권한 필요

		sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;			// Vulkan의 파이프라인에서 가장 상단에 위치한 첫 번째 단계로, 어떠한 작업도 진행되지 않은 상태
		destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;			// 데이터 복사 단계
	} else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
		// 이미지 복사가 완료되고 읽기를 수행하기 위해 Fragment shader 작업 대기
		barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;		// 쓰기 권한 필요
		barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;			// 읽기 권한 필요

		sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;				// 데이터 복사 단계
		destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;	// Fragment shader 단계
	} else {
		throw std::invalid_argument("unsupported layout transition!");
	}

	// 베리어를 커맨드 버퍼에 기록
	vkCmdPipelineBarrier(
		commandBuffer,						// 베리어를 기록할 커맨드 버퍼
		sourceStage, destinationStage,		// sourceStage 단계가 끝나면 베리어 작업 시작, 베리어 작업이 끝나기 전에 destinationStage에 돌입한 다른 작업들 모두 대기
		0,									// 의존성 플래그
		0, nullptr,							// 메모리 베리어 (개수 + 베리어 포인터)
		0, nullptr,							// 버퍼 베리어   (개수 + 베리어 포인터)
		1, &barrier							// 이미지 베리어 (개수 + 베리어 포인터)
	);

	endSingleTimeCommands(graphicsQueue, commandPool, commandBuffer);	// 커맨드 버퍼 기록 종료

}

// 한 번만 실행할 커맨드 버퍼 생성 및 기록 시작
VkCommandBuffer TextureImage::beginSingleTimeCommands(VkCommandPool commandPool) {
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
void TextureImage::endSingleTimeCommands(VkQueue graphicsQueue, VkCommandPool commandPool, VkCommandBuffer commandBuffer) {
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

// 텍스처 이미지 뷰 생성
void TextureImage::createTextureImageView() {
	// SRGB 총 4바이트 포맷으로 된 이미지 뷰 생성
	createImageView(VK_IMAGE_ASPECT_COLOR_BIT, mipLevels);
}

// 텍스처를 위한 샘플러 생성 함수
void TextureImage::createTextureSampler(VkPhysicalDevice physicalDevice) {
	// GPU의 속성 정보를 가져오는 함수
	VkPhysicalDeviceProperties properties{};
	vkGetPhysicalDeviceProperties(physicalDevice, &properties);

	// 샘플러 생성시 필요한 구조체 
	VkSamplerCreateInfo samplerInfo{};
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	samplerInfo.magFilter = VK_FILTER_LINEAR;							// 확대시 필터링 적용 설정 (현재 선형 보간 필터링 적용)
	samplerInfo.minFilter = VK_FILTER_LINEAR;							// 축소시 필터링 적용 설정 (현재 선형 보간 필터링 적용)
	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;			// 텍스처 좌표계의 U축(너비)에서 범위를 벗어난 경우 래핑 모드 설정 (현재 반복 설정) 
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;			// 텍스처 좌표계의 V축(높이)에서 범위를 벗어난 경우 래핑 모드 설정 (현재 반복 설정) 
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;			// 텍스처 좌표계의 W축(깊이)에서 범위를 벗어난 경우 래핑 모드 설정 (현재 반복 설정) 
	samplerInfo.anisotropyEnable = VK_TRUE;								// 이방성 필터링 적용 여부 설정 (경사진 곳이나 먼 곳의 샘플을 늘려 좀 더 정확한 값을 가져오는 방법)
	samplerInfo.maxAnisotropy = properties.limits.maxSamplerAnisotropy;	// GPU가 지원하는 최대의 이방성 필터링 제한 설정
	samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;			// 래핑 모드가 VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER 일때 텍스처 외부 색 지정
	samplerInfo.unnormalizedCoordinates = VK_FALSE;						// VK_TRUE로 설정시 텍스처 좌표가 0 ~ 1의 정규화된 좌표가 아닌 실제 텍셀의 좌표 범위로 바뀜
	samplerInfo.compareEnable = VK_FALSE;								// 비교 연산 사용할지 결정 (보통 쉐도우 맵같은 경우 깊이 비교 샘플링에서 사용됨)
	samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;						// 비교 연산에 사용할 연산 지정
	samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;				// mipmap 사용시 mipmap 간 보간 방식 결정 (현재 선형 보간)
	samplerInfo.minLod = 0.0f;											// 최소 level을 0으로 설정 (가장 높은 해상도의 mipmap 을 사용가능하게 허용)
	samplerInfo.maxLod = static_cast<float>(mipLevels);					// 최대 level을 mipLevel로 설정 (VK_LOD_CLAMP_NONE 설정시 제한 해제)
	samplerInfo.mipLodBias = 0.0f;										// Mipmap 레벨 오프셋(Bias)을 설정 
																		// Mipmap을 일부러 더 높은(더 큰) 레벨로 사용하거나 낮은(더 작은) 레벨로 사용하고 싶을 때 사용.

	// 샘플러 생성
	if (vkCreateSampler(device, &samplerInfo, nullptr, &sampler) != VK_SUCCESS) {
		throw std::runtime_error("failed to create texture sampler!");
	}
}

void TextureImage::clear()
{
	vkDestroySampler(device, sampler, nullptr);					// 샘플러 삭제
	vkDestroyImageView(device, imageView, nullptr);				// 텍스처 이미지뷰 삭제
	vkDestroyImage(device, image, nullptr);						// 텍스처 객체 삭제
	vkFreeMemory(device, imageMemory, nullptr);					// 텍스처에 할당된 메모리 삭제
}

VkSampler TextureImage::getSampler()
{
	return sampler;
}