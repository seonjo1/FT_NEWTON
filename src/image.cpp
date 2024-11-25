#include "../include/image.h"

void Image::clear(VkDevice device)
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
	VkDevice device = deviceManager->getLogicalDevice();

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
void Image::createImageView(VkDevice device, VkImageAspectFlags aspectFlags, uint32_t mipLevels) {
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
	imageFormat = format;
	createImage(deviceManager, swapChainExtent.width, swapChainExtent.height, 1, deviceManager->getMsaaSamples(), VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	createImageView(deviceManager->getLogicalDevice(), VK_IMAGE_ASPECT_COLOR_BIT, 1);
}

std::unique_ptr<DepthImage> DepthImage::create(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent)
{
	std::unique_ptr<DepthImage> depthImage(new DepthImage());
	depthImage->init(deviceManager, format, swapChainExtent);
	return depthImage;
}

// Depth test에 쓰일 image, imageView 준비
void DepthImage::init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent) {
	imageFormat = format;
	// depth image의 format 결정
	createImage(deviceManager, swapChainExtent.width, swapChainExtent.height, 1, deviceManager->getMsaaSamples(), VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	createImageView(deviceManager->getLogicalDevice(), VK_IMAGE_ASPECT_DEPTH_BIT, 1);
}



// std::unique_ptr<Texture> create()
// {
// 	std::unique_ptr<DepthImage> colorImage(new DepthImage());
// 	colorImage->init(deviceManager, colorFormat);
// 	return colorImage;
// }
