#include "../include/swapChainManager.h"
#include "../include/image.h"

std::unique_ptr<SwapChainManager> SwapChainManager::create(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface)
{
	std::unique_ptr<SwapChainManager> swapChainManager(new SwapChainManager());
	swapChainManager->init(window, deviceManager, surface);
	return swapChainManager;
}

/*
	[프레임 버퍼 생성]
	프레임 버퍼를 생성하고 SwapChain의 이미지를 attachment로 설정 
	(depth buffer 같은 image가 더 필요하면 따로 image를 생성해서 attachment에 추가해야 함)
*/
void SwapChainManager::createFramebuffers(DeviceManager* deviceManager, VkRenderPass renderPass)
{
	// colorImage, depthImage 생성
	colorImage = Image::create(deviceManager, swapChainImageFormat, swapChainExtent);
	depthImage = DepthImage::create(deviceManager, deviceManager->findDepthFormat(), swapChainExtent);
	
	// 프레임 버퍼 배열 초기화
	swapChainFramebuffers.resize(swapChainImageViews.size());

	// 이미지 뷰마다 프레임 버퍼 1개씩 생성
	for (size_t i = 0; i < swapChainImageViews.size(); i++) {
		std::array<VkImageView, 3> attachments = {
			colorImage->getImageView(),
			depthImage->getImageView(),
			swapChainImageViews[i]
		};

		VkFramebufferCreateInfo framebufferInfo{};
		framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		framebufferInfo.renderPass = renderPass; 										// 렌더 패스 등록
		framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size()); 	// attachment 개수
		framebufferInfo.pAttachments = attachments.data();								// attachment 등록
		framebufferInfo.width = swapChainExtent.width;									// 프레임 버퍼 width
		framebufferInfo.height = swapChainExtent.height;								// 프레임 버퍼 height
		framebufferInfo.layers = 1;														// 레이어 수

		if (vkCreateFramebuffer(deviceManager->getLogicalDevice(), &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS) {
			throw std::runtime_error("failed to create framebuffer!");
		}
	}
}

void SwapChainManager::init(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface)
{
	device = deviceManager->getLogicalDevice();
	createSwapChain(window, deviceManager, surface);
	createImageViews();
}

/*
	[스왑 체인 생성]
	스왑 체인의 역할
	1. 다수의 프레임 버퍼(이미지) 관리
	2. 이중 버퍼링 및 삼중 버퍼링 (다수의 버퍼를 이용하여 화면에 프레임 전환시 딜레이 최소화)
	3. 프레젠테이션 모드 관리 (화면에 프레임을 표시하는 방법 설정 가능)
	4. 화면과 GPU작업의 동기화 (GPU가 이미지를 생성하는 작업과 화면이 이미지를 띄우는 작업 간의 동기화) 
*/ 
void SwapChainManager::createSwapChain(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface) {
	
	SwapChainSupportDetails swapChainSupport = deviceManager->getSwapChainSupport();
	QueueFamilyIndices queueFamilyIndices = deviceManager->getQueueFamilyIndices();

	// 서피스 포맷 선택
	VkSurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.formats);
	// 프레젠테이션 모드 선택
	VkPresentModeKHR presentMode = chooseSwapPresentMode(swapChainSupport.presentModes);
	// 스왑 범위 선택 (스왑 체인의 이미지 해상도 결정)
	VkExtent2D extent = chooseSwapExtent(window, swapChainSupport.capabilities);

	// 스왑 체인에서 필요한 이미지 수 결정 (최소 이미지 수 + 1)
	uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;

	// 만약 필요한 이미지 수가 최댓값을 넘으면 clamp
	if (swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount) {
		imageCount = swapChainSupport.capabilities.maxImageCount;
	}
	
	// SwapChain 정보 생성
	VkSwapchainCreateInfoKHR createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	createInfo.surface = surface;

	// 이미지 개수의 최솟값 설정 (최댓값은 아까 봤던 이미지 최댓값으로 들어감)
	createInfo.minImageCount = imageCount;
	// 이미지 정보 입력
	createInfo.imageFormat = surfaceFormat.format;
	createInfo.imageColorSpace = surfaceFormat.colorSpace;
	createInfo.imageExtent = extent;
	createInfo.imageArrayLayers = 1; // 한 번의 렌더링에 n 개의 결과가 생긴다. (스테레오 3D, cubemap 이용시 여러 개 레이어 사용)
	createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT; // 기본 렌더링에만 사용하는 플래그 (만약 렌더링 후 2차 가공 필요시 다른 플래그 사용)

	uint32_t queueFamilyIndicesArray[] = {queueFamilyIndices.graphicsFamily.value(), queueFamilyIndices.presentFamily.value()};

	// 큐패밀리 정보 등록
	if (queueFamilyIndices.graphicsFamily != queueFamilyIndices.presentFamily) {
		// [그래픽스 큐패밀리와 프레젠트 큐패밀리가 서로 다른 큐인 경우]
		// 하나의 이미지에 두 큐패밀리가 동시에 접근할 수 있게 하여 성능을 높인다.
		// (큐패밀리가 이미지에 접근할 때 다른 큐패밀리가 접근했는지 확인하는 절차 삭제)
		// 그러나 실제로 동시에 접근하면 안 되므로 순서를 프로그래머가 직접 조율해야 한다. 
		createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT; // 동시 접근 허용
		createInfo.queueFamilyIndexCount = 2;                     // 큐패밀리 개수 등록
		createInfo.pQueueFamilyIndices = queueFamilyIndicesArray;      // 큐패밀리 인덱스 배열 등록
	} else {
		// [그래픽스 큐패밀리와 프레젠트 큐패밀리가 서로 같은 큐인 경우]
		// 어처피 1개의 큐패밀리만 존재하므로 큐패밀리의 이미지 독점을 허용한다.
		createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;  // 큐패밀리의 독점 접근 허용
	}

	createInfo.preTransform = swapChainSupport.capabilities.currentTransform;  // 스왑 체인 이미지를 화면에 표시할때 적용되는 변환 등록
	createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;  // 렌더링 결과의 알파 값을 window에도 적용시킬지 설정 (현재는 알파블랜딩 없는 설정)
	createInfo.presentMode = presentMode; // 프레젠트 모드 설정
	createInfo.clipped = VK_TRUE; // 실제 컴퓨터 화면에 보이지 않는 부분을 렌더링 할 것인지 설정 (VK_TRUE = 렌더링 하지 않겠다)

	createInfo.oldSwapchain = VK_NULL_HANDLE; // 재활용할 이전 스왑체인 설정 (만약 설정한다면 새로운 할당을 하지 않고 가능한만큼 이전 스왑체인 리소스 재활용)

	/* 
	[스왑 체인 생성] 
	스왑 체인 생성시 이미지들도 설정대로 만들어지고, 
	만약 렌더링에 필요한 추가 이미지가 있으면 따로 만들어야 함 
	*/
	if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS) {
		throw std::runtime_error("failed to create swap chain!");
	}

	// 스왑 체인 이미지 개수 저장
	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
	// 스왑 체인 이미지 개수만큼 vector 초기화 
	swapChainImages.resize(imageCount);
	// 이미지 개수만큼 vector에 스왑 체인의 이미지 핸들 채우기 
	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());

	// 스왑 체인의 이미지 포맷 저장
	swapChainImageFormat = surfaceFormat.format;
	// 스왑 체인의 이미지 크기 저장
	swapChainExtent = extent;
}

/* 
지원하는 포맷중 선호하는 포맷 1개 반환
선호하는 포맷이 없을 시 가장 앞에 있는 포맷 반환
*/
VkSurfaceFormatKHR SwapChainManager::chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats) {
	for (const auto& availableFormat : availableFormats) {
		// 만약 선호하는 포맷이 존재할 경우 그 포맷을 반환
		if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB &&
			availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
			return availableFormat;
		}
	}
	// 선호하는 포맷이 없는 경우 첫 번째 포맷 반환
	return availableFormats[0];
}

/*
지원하는 프레젠테이션 모드 중 선호하는 모드 선택
선호하는 모드가 없을 시 기본 값인 VK_PRESENT_MODE_FIFO_KHR 반환
*/
VkPresentModeKHR SwapChainManager::chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes) {
	for (const auto& availablePresentMode : availablePresentModes) {
		// 선호하는 mode가 존재하면 해당 mode 반환 
		if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
			return availablePresentMode;
		}
	}
	// 선호하는 mode가 존재하지 않으면 기본 값인 VK_PRESENT_MODE_FIFO_KHR 반환
	return VK_PRESENT_MODE_FIFO_KHR;
}

// 스왑 체인 이미지의 해상도 결정
VkExtent2D SwapChainManager::chooseSwapExtent(GLFWwindow* window, const VkSurfaceCapabilitiesKHR& capabilities) {
	// 만약 currentExtent의 width가 std::numeric_limits<uint32_t>::max()가 아니면, 시스템이 이미 권장하는 스왑체인 크기를 제공하는 것
	if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
		return capabilities.currentExtent; // 권장 사항 사용
	} else {
		// 그렇지 않은 경우, 창의 현재 프레임 버퍼 크기를 사용하여 스왑체인 크기를 결정
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);

		VkExtent2D actualExtent = {
			static_cast<uint32_t>(width),
			static_cast<uint32_t>(height)
		};

		// width, height를 capabilites의 min, max 범위로 clamping
		actualExtent.width = std::clamp(actualExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
		actualExtent.height = std::clamp(actualExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

		return actualExtent;
	}
}

/*
	[이미지 뷰 생성]
	이미지 뷰란 VKImage에 대한 접근 방식을 정의하는 객체
	GPU가 이미지를 읽을 때만 사용 (이미지를 텍스처로 쓰거나 후처리 하는 등)
	GPU가 이미지에 쓰기 작업할 떄는 x
*/ 
void SwapChainManager::createImageViews() {
	// 이미지의 개수만큼 vector 초기화
	swapChainImageViews.resize(swapChainImages.size());

	// 이미지의 개수만큼 이미지뷰 생성
	for (size_t i = 0; i < swapChainImages.size(); i++) {
		swapChainImageViews[i] = Image::createSwapChainImageView(device, swapChainImages[i], swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT, 1);
	}
}

// FrameBuffer, ImageView, SwapChain 삭제
void SwapChainManager::cleanupSwapChain() {

	// 컬러 버퍼 이미지, 이미지 뷰, 메모리 삭제
	colorImage->clear(device);
	// 깊이 버퍼 이미지, 이미지 뷰, 메모리 삭제 
	depthImage->clear(device);

	// 프레임 버퍼 배열 삭제
	for (auto framebuffer : swapChainFramebuffers) {
		vkDestroyFramebuffer(device, framebuffer, nullptr);
	}
	// 이미지뷰 삭제
	for (auto imageView : swapChainImageViews) {
		vkDestroyImageView(device, imageView, nullptr);
	}
	// 스왑 체인 파괴
	vkDestroySwapchainKHR(device, swapChain, nullptr);
}


/*
	변경된 window 크기에 맞게 SwapChain, ImageView, FrameBuffer 재생성
*/
void SwapChainManager::recreateSwapChain(GLFWwindow* window, DeviceManager* deviceManager, VkSurfaceKHR surface, VkRenderPass renderPass)
{
	// 현재 프레임버퍼 사이즈 체크
	int width = 0, height = 0;
	glfwGetFramebufferSize(window, &width, &height);
	
	// 현재 프레임 버퍼 사이즈가 0이면 다음 이벤트 호출까지 대기
	while (width == 0 || height == 0) {
		glfwGetFramebufferSize(window, &width, &height);
		glfwWaitEvents(); // 다음 이벤트 발생 전까지 대기하여 CPU 사용률을 줄이는 함수 
	}

	// 모든 GPU 작업 종료될 때까지 대기 (사용중인 리소스를 건들지 않기 위해)
	vkDeviceWaitIdle(device);

	// 스왑 체인 관련 리소스 정리
	cleanupSwapChain();

	// 현재 window 크기에 맞게 SwapChain, DepthResource, ImageView, FrameBuffer 재생성
	createSwapChain(window,  deviceManager, surface);
	createImageViews();
	createFramebuffers(deviceManager, renderPass);
}

VkSwapchainKHR SwapChainManager::getSwapChain()
{
	return swapChain;
}

VkFormat SwapChainManager::getSwapChainImageFormat()
{
	return swapChainImageFormat;
}

VkExtent2D SwapChainManager::getSwapChainExtent()
{
	return swapChainExtent;
}

std::vector<VkFramebuffer>& SwapChainManager::getFramebuffers()
{
	return swapChainFramebuffers;
}
