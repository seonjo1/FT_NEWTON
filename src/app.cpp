#include "../include/common.h"
#include "../include/vulkanInstance.h"
#include "../include/deviceManager.h"
#include "../include/swapChainManager.h"
#include "../include/image.h"
#include "../include/renderer.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

#include <chrono>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <optional>

// texture 경로
const std::string MODEL_PATH = "models/viking_room.obj";
const std::string TEXTURE_PATH = "textures/viking_room.png";

// 동시에 처리할 최대 프레임 수
const int MAX_FRAMES_IN_FLIGHT = 2;

struct UniformBufferObject {
	alignas(16) glm::mat4 model;
	alignas(16) glm::mat4 view;
	alignas(16) glm::mat4 proj;
};

class App {
public:
	void run() {
		initWindow();
		initVulkan();
		mainLoop();
		cleanup();
	}

private:
	GLFWwindow* window;

	std::unique_ptr<VulkanInstance> vulkanInstance;
		VkInstance instance;
		VkSurfaceKHR surface;

	std::unique_ptr<DeviceManager> deviceManager;
		VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
		VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT;
		VkDevice device;
		VkQueue graphicsQueue;
		VkQueue presentQueue;
		QueueFamilyIndices queueFamilyIndices;	
		SwapChainSupportDetails swapChainSupport;

	std::unique_ptr<SwapChainManager> swapChainManager;
		VkSwapchainKHR swapChain;
		VkFormat swapChainImageFormat;
		VkExtent2D swapChainExtent;
		std::vector<VkFramebuffer> swapChainFramebuffers;

	// Renderer
	std::unique_ptr<Renderer> renderer;
		VkRenderPass renderPass;
		VkDescriptorSetLayout descriptorSetLayout;
		VkPipelineLayout pipelineLayout;
		VkPipeline graphicsPipeline;

	VkCommandPool commandPool;
	
	uint32_t mipLevels;
	VkImage textureImage;
	VkDeviceMemory textureImageMemory;
	VkImageView textureImageView;
    VkSampler textureSampler;

	std::vector<Vertex> vertices;
	std::vector<uint32_t> indices;
	VkBuffer vertexBuffer;
	VkDeviceMemory vertexBufferMemory;
	VkBuffer indexBuffer;
	VkDeviceMemory indexBufferMemory;

	std::vector<VkBuffer> uniformBuffers;
	std::vector<VkDeviceMemory> uniformBuffersMemory;
	std::vector<void*> uniformBuffersMapped;

	VkDescriptorPool descriptorPool;
	std::vector<VkDescriptorSet> descriptorSets;
	
	std::vector<VkCommandBuffer> commandBuffers;

	std::vector<VkSemaphore> imageAvailableSemaphores;
	std::vector<VkSemaphore> renderFinishedSemaphores;
	std::vector<VkFence> inFlightFences;
	uint32_t currentFrame = 0;

	bool framebufferResized = false;

	// glfw 실행, window 생성, 콜백 함수 등록
	void initWindow() {
		glfwInit();

		// glfw로 창관리만 하고 렌더링 API는 안 쓴다는 설정
		// 렌더링 및 그래픽 처리는 외부 API인 Vulkan으로 함
		glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

		window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Vulkan", nullptr, nullptr);
		// window에 현재 App 객체를 바인딩
		glfwSetWindowUserPointer(window, this);
		// 프레임버퍼 사이즈 변경 콜백 함수 등록
		glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);
	}

	static void framebufferResizeCallback(GLFWwindow* window, int width, int height) {
		// window에 바인딩된 객체 호출 및 framebufferResized = true 설정
		auto app = reinterpret_cast<App*>(glfwGetWindowUserPointer(window));
		app->framebufferResized = true;
	}

	// 렌더링을 위한 초기 setting
	void initVulkan() {
		vulkanInstance = VulkanInstance::create(window);
			instance = vulkanInstance->getInstance();
			surface = vulkanInstance->getSurface();

		deviceManager = DeviceManager::create(vulkanInstance.get());
			physicalDevice = deviceManager->getPhysicalDevice();
			device = deviceManager->getLogicalDevice();
			queueFamilyIndices = deviceManager->getQueueFamilyIndices();
			graphicsQueue = deviceManager->getGraphicsQueue();
			presentQueue = deviceManager->getPresentQueue();
			msaaSamples = deviceManager->getMsaaSamples();
			swapChainSupport = deviceManager->getSwapChainSupport();
		
		// SwapChainManager 클래스
		swapChainManager = SwapChainManager::create(window, deviceManager.get(), vulkanInstance->getSurface());
			swapChain = swapChainManager->getSwapChain();
			swapChainImageFormat = swapChainManager->getSwapChainImageFormat();
			swapChainExtent = swapChainManager->getSwapChainExtent();

		// Command 클래스
		createCommandPool();

		// Renderer 클래스
		renderer = Renderer::create(deviceManager.get(), swapChainManager->getSwapChainImageFormat());
			renderPass = renderer->getRenderPass();
			descriptorSetLayout = renderer->getDescriptorSetLayout();
			pipelineLayout = renderer->getPipelineLayout();
			graphicsPipeline = renderer->getGraphicsPipeline();
			createCommandBuffers();


		swapChainManager->createFramebuffers(deviceManager.get(), renderPass);
		swapChainFramebuffers = swapChainManager->getFramebuffers();


		// Model 클래스
			loadModel();
			createVertexBuffer();
			createIndexBuffer();
			createUniformBuffers();

			// Texture 클래스
			createTextureImage();
			createTextureImageView();
			createTextureSampler();

		// Descriptor 클래스		
		createDescriptorPool();
		createDescriptorSets();

		// 동기화 클래스
		createSyncObjects();
	}

	/*
		렌더링 루프 실행	
	*/
	void mainLoop() {
		while (!glfwWindowShouldClose(window)) {
			glfwPollEvents();
			drawFrame();
		}
		vkDeviceWaitIdle(device);  // 종료시 실행 중인 GPU 작업을 전부 기다림
	}

	/*
		[사용한 자원들 정리]
	*/
	void cleanup() {
		// 스왑 체인 파괴
		swapChainManager->cleanupSwapChain();

		renderer->clear();

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
			// 매핑된 거 해제 안하나?????????????????????
            vkDestroyBuffer(device, uniformBuffers[i], nullptr);	// 유니폼 버퍼 객체 삭제
            vkFreeMemory(device, uniformBuffersMemory[i], nullptr);	// 유니폼 버퍼에 할당된 메모리 삭제
        }

		vkDestroyDescriptorPool(device, descriptorPool, nullptr);			// 디스크립터 풀 삭제
 
		vkDestroySampler(device, textureSampler, nullptr);					// 샘플러 삭제
		vkDestroyImageView(device, textureImageView, nullptr);				// 텍스처 이미지뷰 삭제

		vkDestroyImage(device, textureImage, nullptr);						// 텍스처 객체 삭제
		vkFreeMemory(device, textureImageMemory, nullptr);					// 텍스처에 할당된 메모리 삭제

		vkDestroyBuffer(device, indexBuffer, nullptr);				// 인덱스 버퍼 객체 삭제
		vkFreeMemory(device, indexBufferMemory, nullptr);			// 인덱스 버퍼에 할당된 메모리 삭제
		
		vkDestroyBuffer(device, vertexBuffer, nullptr);				// 버텍스 버퍼 객체 삭제
		vkFreeMemory(device, vertexBufferMemory, nullptr);			// 버텍스 버퍼에 할당된 메모리 삭제

		// 세마포어, 펜스 파괴
		for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
			vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
			vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
			vkDestroyFence(device, inFlightFences[i], nullptr);
		}

		vkDestroyCommandPool(device, commandPool, nullptr); 	  	// 커맨드 풀 파괴


		vulkanInstance->clear();

		glfwDestroyWindow(window);                              	// 윈도우 파괴
		glfwTerminate();									        // glfw 종료
	}


	/*
		[커맨드 풀 생성]
		커맨드 풀이란?
		1. 커맨드 버퍼들을 관리한다.
		2. 큐 패밀리당 1개의 커맨드 풀이 필요하다.
	*/
	void createCommandPool() {
		VkCommandPoolCreateInfo poolInfo{};
		poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
		poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT; 		// 커맨드 버퍼를 개별적으로 재설정할 수 있도록 설정 
																				// (이게 아니면 커맨드 풀의 모든 커맨드 버퍼 설정이 한 번에 이루어짐)
		poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value(); 	// 그래픽스 큐 인덱스 등록 (대응시킬 큐 패밀리 등록)

		// 커맨드 풀 생성
		if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS) {
			throw std::runtime_error("failed to create command pool!");
		}
	}

	bool hasStencilComponent(VkFormat format) {
		return format == VK_FORMAT_D32_SFLOAT_S8_UINT || format == VK_FORMAT_D24_UNORM_S8_UINT;
	}

	void createTextureImage() {
		int texWidth, texHeight, texChannels;
		stbi_uc* pixels = stbi_load(TEXTURE_PATH.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha); // 알파 채널을 포함하여 rgba 픽셀로 이미지 저장
		VkDeviceSize imageSize = texWidth * texHeight * 4;  // 이미지 크기 (픽셀당 4byte)
        mipLevels = static_cast<uint32_t>(std::floor(std::log2(std::max(texWidth, texHeight)))) + 1; // mipLevel 설정


		if (!pixels) {
			// 로드 실패시 오류 처리
			throw std::runtime_error("failed to load texture image!");
		}

		// 스테이징 버퍼 생성
		VkBuffer stagingBuffer;
		VkDeviceMemory stagingBufferMemory;
		createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

		// 스테이징 버퍼에 이미지 데이터 복사
		void* data;
		vkMapMemory(device, stagingBufferMemory, 0, imageSize, 0, &data);
		memcpy(data, pixels, static_cast<size_t>(imageSize));
		vkUnmapMemory(device, stagingBufferMemory);

		// 이미지 데이터 해제
		stbi_image_free(pixels);

		// 이미지 객체 생성
		createImage(texWidth, texHeight, mipLevels, VK_SAMPLE_COUNT_1_BIT, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, textureImage, textureImageMemory);

		// Top stage 끝나고 베리어를 이용한 이미지 전환 설정 
		// (같은 작업 큐에서 Transfer 단계 들어가는 다른 작업들 해당 베리어 작업이 끝날때까지 stop)
		transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, mipLevels);
		
		// 커맨드 버퍼를 이용한 버퍼 -> 이미지 데이터 복사 실행
		copyBufferToImage(stagingBuffer, textureImage, static_cast<uint32_t>(texWidth), static_cast<uint32_t>(texHeight));

		// Transfer 끝나고 베리어를 이용한 이미지 전환 설정
		// (같은 작업 큐에서 Fragment shader 단계 들어가는 다른 작업들 해당 베리어 작업이 끝날때까지 stop)
		// mipmap 생성에서 하는걸로 변경
		// transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, mipLevels);

		// 스테이징 버퍼 삭제
		vkDestroyBuffer(device, stagingBuffer, nullptr);
		vkFreeMemory(device, stagingBufferMemory, nullptr);

		// mipmap 생성
        generateMipmaps(textureImage, VK_FORMAT_R8G8B8A8_SRGB, texWidth, texHeight, mipLevels);
	}

	void generateMipmaps(VkImage image, VkFormat imageFormat, int32_t texWidth, int32_t texHeight, uint32_t mipLevels) {
		// 이미지 포맷이 선형 필터링을 사용한 Blit 작업을 지원하는지 확인
		VkFormatProperties formatProperties;
		vkGetPhysicalDeviceFormatProperties(physicalDevice, imageFormat, &formatProperties);

		if (!(formatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT)) {
			throw std::runtime_error("texture image format does not support linear blitting!");
		}

		// 커맨드 버퍼 생성 및 기록 시작
		VkCommandBuffer commandBuffer = beginSingleTimeCommands();

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

		endSingleTimeCommands(commandBuffer);
	}


	// 텍스처 이미지 뷰 생성
	void createTextureImageView() {
		// SRGB 총 4바이트 포맷으로 된 이미지 뷰 생성
		textureImageView = createImageView(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_ASPECT_COLOR_BIT, mipLevels);
	}

	// 텍스처를 위한 샘플러 생성 함수
	void createTextureSampler() {
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
		if (vkCreateSampler(device, &samplerInfo, nullptr, &textureSampler) != VK_SUCCESS) {
			throw std::runtime_error("failed to create texture sampler!");
		}
	}

	// .obj 파일을 읽고 vertices, indices 채우기
	void loadModel() {
		Assimp::Importer importer;
		// scene 구조체 받아오기
		auto scene = importer.ReadFile(MODEL_PATH, aiProcess_Triangulate | aiProcess_FlipUVs);

		// scene load 오류 처리
		if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
		{
			throw std::runtime_error("failed to load obj file!");
		}
		// node 데이터 처리
		processNode(scene->mRootNode, scene);
	}

	// node에 포함된 mesh들의 데이터 처리
	void processNode(aiNode *node, const aiScene *scene)
	{
		// node에 포함된 mesh들 순회
		for (uint32_t i = 0; i < node->mNumMeshes; i++)
		{
			// 현재 처리할 mesh 찾기
			auto meshIndex = node->mMeshes[i];
			auto mesh = scene->mMeshes[meshIndex];
			// 현재 mesh 데이터 처리
			processMesh(mesh, scene);
		}

		// 자식 노드 처리
		for (uint32_t i = 0; i < node->mNumChildren; i++)
			processNode(node->mChildren[i], scene);
	}

	// mesh의 vetex, index 데이터 처리
	void processMesh(aiMesh *mesh, const aiScene *scene)
	{
		// mesh의 vertex 정보 저장
		vertices.resize(mesh->mNumVertices);
		for (uint32_t i = 0; i < mesh->mNumVertices; i++)
		{
			Vertex& v = vertices[i];
			v.pos = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
			v.texCoord = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
			v.color = {1.0f, 1.0f, 1.0f};
		}

		// mesh의 index 정보 저장
		indices.resize(mesh->mNumFaces * 3);
		// face의 개수 = triangle 개수
		for (uint32_t i = 0; i < mesh->mNumFaces; i++)
		{
			indices[3 * i] = mesh->mFaces[i].mIndices[0];
			indices[3 * i + 1] = mesh->mFaces[i].mIndices[1];
			indices[3 * i + 2] = mesh->mFaces[i].mIndices[2];
		}
	}

	// 이미지 레이아웃, 접근 권한을 변경할 수 있는 베리어를 커맨드 버퍼에 기록
	void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels) {
		VkCommandBuffer commandBuffer = beginSingleTimeCommands();			// 커맨드 버퍼 생성 및 기록 시작

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

		endSingleTimeCommands(commandBuffer);	// 커맨드 버퍼 기록 종료
	}

	// 커맨드 버퍼 제출을 통해 버퍼 -> 이미지 데이터 복사 
	void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height) {
		// 커맨드 버퍼 생성 및 기록 시작
		VkCommandBuffer commandBuffer = beginSingleTimeCommands();

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
		endSingleTimeCommands(commandBuffer);
	}

	/*
		[버텍스 버퍼 생성]
		1. 스테이징 버퍼 생성
		2. 스테이징 버퍼에 정점 정보 복사
		3. 버텍스 버퍼 생성
		4. 스테이징 버퍼의 정점 정보를 버텍스 버퍼에 복사
		5. 스테이징 버퍼 리소스 해제
	*/ 
	void createVertexBuffer() {
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
		createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

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
		createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

		// [스테이징 버퍼에서 버텍스 버퍼로 메모리 이동]
		copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

		// 스테이징 버퍼와 할당된 메모리 해제
		vkDestroyBuffer(device, stagingBuffer, nullptr);
		vkFreeMemory(device, stagingBufferMemory, nullptr);
	}

	/*
		[인덱스 버퍼 생성]
		버텍스 버퍼 생성 과정과 같음
	*/
	void createIndexBuffer() {
		VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();

		VkBuffer stagingBuffer;
		VkDeviceMemory stagingBufferMemory;
		createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

		void* data;
		vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
		memcpy(data, indices.data(), (size_t) bufferSize);
		vkUnmapMemory(device, stagingBufferMemory);

		// [버텍스 버퍼 생성]
		// 속성)
		// 		VK_BUFFER_USAGE_INDEX_BUFFER_BIT : 버퍼를 인덱스 데이터를 저장하고 처리하는 용도로 설정.
		// 		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT : GPU 전용 메모리에 데이터를 저장하여, GPU가 최적화된 방식으로 접근할 수 있게 함.
		createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

		copyBuffer(stagingBuffer, indexBuffer, bufferSize);

		vkDestroyBuffer(device, stagingBuffer, nullptr);
		vkFreeMemory(device, stagingBufferMemory, nullptr);
	}

	// 유니폼 버퍼 생성
	void createUniformBuffers() {
		// 유니폼 버퍼에 저장 될 구조체의 크기
		VkDeviceSize bufferSize = sizeof(UniformBufferObject);

		// 각 요소들을 동시에 처리 가능한 최대 프레임 수만큼 만들어 둔다.
		uniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);		// 유니폼 버퍼 객체
		uniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);	// 유니폼 버퍼에 할당할 메모리
		uniformBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);	// GPU 메모리에 매핑할 CPU 메모리 포인터

		for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
			// 유니폼 버퍼 객체 생성 + 메모리 할당 + 바인딩
			createBuffer(bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, uniformBuffers[i], uniformBuffersMemory[i]);
			// GPU 메모리 CPU 가상 포인터에 매핑
			vkMapMemory(device, uniformBuffersMemory[i], 0, bufferSize, 0, &uniformBuffersMapped[i]);
		}
	}

	// 디스크립터 풀 생성
	void createDescriptorPool() {
		
		// 디스크립터 풀의 타입별 디스크립터 개수를 설정하는 구조체
        std::array<VkDescriptorPoolSize, 2> poolSizes{};
        poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;							// 유니폼 버퍼 설정
        poolSizes[0].descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);		// 유니폼 버퍼 디스크립터 최대 개수 설정
        poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;					// 샘플러 설정
        poolSizes[1].descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);		// 샘플러 디스크립터 최대 개수 설정

		// 디스크립터 풀을 생성할 때 필요한 설정 정보를 담는 구조체
		VkDescriptorPoolCreateInfo poolInfo{};
		poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());			// 디스크립터 poolSize 구조체 개수
        poolInfo.pPoolSizes = poolSizes.data();										// 디스크립터 poolSize 구조체 배열
		poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);				// 풀에 존재할 수 있는 총 디스크립터 셋 개수

		// 디스크립터 풀 생성
		if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
			throw std::runtime_error("failed to create descriptor pool!");
		}
	}

	// 디스크립터 셋 할당 및 업데이트 하여 리소스 바인딩
	void createDescriptorSets() {
		// 디스크립터 셋 레이아웃 벡터 생성 (기존 만들어놨던 디스크립터 셋 레이아웃 객체 이용)
		std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, descriptorSetLayout);

		// 디스크립터 셋 할당에 필요한 정보를 설정하는 구조체
		VkDescriptorSetAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		allocInfo.descriptorPool = descriptorPool;										// 디스크립터 셋을 할당할 디스크립터 풀 지정
		allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);		// 할당할 디스크립터 셋 개수 지정
		allocInfo.pSetLayouts = layouts.data();											// 할당할 디스크립터 셋 의 레이아웃을 정의하는 배열 

		descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);									// 디스크립터 셋을 저장할 벡터 크기 설정
		
		// 디스크립터 풀에 디스크립터 셋 할당
		if (vkAllocateDescriptorSets(device, &allocInfo, descriptorSets.data()) != VK_SUCCESS) {
			throw std::runtime_error("failed to allocate descriptor sets!");
		}

		// 디스크립터 셋마다 디스크립터 설정 진행
		for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
			// 디스크립터 셋에 바인딩할 버퍼 정보 
			VkDescriptorBufferInfo bufferInfo{};
			bufferInfo.buffer = uniformBuffers[i];								// 바인딩할 버퍼
			bufferInfo.offset = 0;												// 버퍼에서 데이터 시작 위치 offset
			bufferInfo.range = sizeof(UniformBufferObject);						// 셰이더가 접근할 버퍼 크기

            VkDescriptorImageInfo imageInfo{};								
            imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;	// 이미지의 레이아웃
            imageInfo.imageView = textureImageView;								// 셰이더에서 사용할 이미지 뷰
            imageInfo.sampler = textureSampler;									// 이미지 샘플링에 사용할 샘플러 설정

			// 디스크립터 셋 바인딩 및 업데이트
			std::array<VkWriteDescriptorSet, 2> descriptorWrites{};

			descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			descriptorWrites[0].dstSet = descriptorSets[i];										// 업데이트 할 디스크립터 셋
			descriptorWrites[0].dstBinding = 0;													// 업데이트 할 바인딩 포인트
			descriptorWrites[0].dstArrayElement = 0;											// 업데이트 할 디스크립터가 배열 타입인 경우 해당 배열의 원하는 index 부터 업데이트 가능 (배열 아니면 0으로 지정)
			descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;				// 업데이트 할 디스크립터 타입
			descriptorWrites[0].descriptorCount = 1;											// 업데이트 할 디스크립터 개수
			descriptorWrites[0].pBufferInfo = &bufferInfo;										// 업데이트 할 버퍼 디스크립터 정보 구조체 배열

			descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			descriptorWrites[1].dstSet = descriptorSets[i];										// 업데이트 할 디스크립터 셋
			descriptorWrites[1].dstBinding = 1;													// 업데이트 할 바인딩 포인트
			descriptorWrites[1].dstArrayElement = 0;											// 업데이트 할 디스크립터가 배열 타입인 경우 해당 배열의 원하는 index 부터 업데이트 가능 (배열 아니면 0으로 지정)
			descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;		// 업데이트 할 디스크립터 타입
			descriptorWrites[1].descriptorCount = 1;											// 업데이트 할 디스크립터 개수
			descriptorWrites[1].pImageInfo = &imageInfo;										// 업데이트 할 버퍼 디스크립터 정보 구조체 배열	

			// 디스크립터 셋을 업데이트 하여 사용할 리소스 바인딩
            vkUpdateDescriptorSets(device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
		}
	}

	/*
		[버퍼 생성]
		1. 버퍼 객체 생성
		2. 버퍼 메모리 할당
		3. 버퍼 객체에 할당한 메모리 바인딩
	*/ 
	void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory) {
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
		allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

		// 버퍼 메모리 할당
		if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
			throw std::runtime_error("failed to allocate buffer memory!");
		}

		// 버퍼 객체에 할당된 메모리를 바인딩 (4번째 매개변수는 할당할 메모리의 offset)
		vkBindBufferMemory(device, buffer, bufferMemory, 0);
	}

	// 한 번만 실행할 커맨드 버퍼 생성 및 기록 시작
	VkCommandBuffer beginSingleTimeCommands() {
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
	void endSingleTimeCommands(VkCommandBuffer commandBuffer) {
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

	// srcBuffer 에서 dstBuffer 로 데이터 복사
	void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size) {
		VkCommandBuffer commandBuffer = beginSingleTimeCommands();

		VkBufferCopy copyRegion{}; 	// 복사할 버퍼 영역을 지정 (크기, src 와 dst의 시작 offset 등)
		copyRegion.size = size;		// 복사할 버퍼 크기 설정
		vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion); // 커맨드 버퍼에 복사 명령 기록

        endSingleTimeCommands(commandBuffer);		
	}


	/*
		[커맨드 버퍼 생성]
		커맨드 버퍼에 GPU에서 실행할 작업을 전부 기록한뒤 제출한다.
		GPU는 해당 커맨드 버퍼의 작업을 알아서 실행하고, CPU는 다른 일을 할 수 있게 된다. (병렬 처리)
	*/
	void createCommandBuffers() {
		// 동시에 처리할 프레임 버퍼 수만큼 커맨드 버퍼 생성
		commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

		// 커맨드 버퍼 설정값 준비
		VkCommandBufferAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		allocInfo.commandPool = commandPool; 								// 커맨드 풀 등록
		allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;					// 큐에 직접 제출할 수 있는 커맨드 버퍼 설정
		allocInfo.commandBufferCount = (uint32_t) commandBuffers.size(); 	// 할당할 커맨드 버퍼의 개수

		// 커맨드 버퍼 할당
		if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) {
			throw std::runtime_error("failed to allocate command buffers!");
		}
	}

	/*
		[커맨드 버퍼에 작업 기록]
		1. 커맨드 버퍼 기록 시작
		2. 렌더패스 시작하는 명령 기록
		3. 파이프라인 설정 명령 기록
		4. 렌더링 명령 기록
		5. 렌더 패스 종료 명령 기록
		6. 커맨드 버퍼 기록 종료
	*/
	void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex) {
		
		// 커맨드 버퍼 기록을 위한 정보 객체
		VkCommandBufferBeginInfo beginInfo{};
		beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

		// GPU에 필요한 작업을 모두 커맨드 버퍼에 기록하기 시작
		if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
			throw std::runtime_error("failed to begin recording command buffer!");
		}

		// 렌더 패스 정보 지정
		VkRenderPassBeginInfo renderPassInfo{};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
		renderPassInfo.renderPass = renderPass;								// 렌더 패스 등록
		renderPassInfo.framebuffer = swapChainFramebuffers[imageIndex];		// 프레임 버퍼 등록
		renderPassInfo.renderArea.offset = {0, 0};							// 렌더링 시작 좌표 등록
		renderPassInfo.renderArea.extent = swapChainExtent;					// 렌더링 width, height 등록 (보통 프레임버퍼, 스왑체인의 크기와 같게 설정)

        std::array<VkClearValue, 2> clearValues{};
        clearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
        clearValues[1].depthStencil = {1.0f, 0};				

		renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());		// clear color 개수 등록
		renderPassInfo.pClearValues = clearValues.data();								// clear color 등록 (첨부한 attachment 개수와 같게 등록)
		
		/* 
			[렌더 패스를 시작하는 명령을 기록] 
			GPU에서 렌더링에 필요한 자원과 설정을 준비 (대략 과정)
			1. 렌더링 자원 초기화 (프레임 버퍼와 렌더 패스에 등록된 attachment layout 초기화)
			2. 서브패스 및 attachment 설정 적용
			3. 렌더링 작업을 위한 컨텍스트 준비 (뷰포트, 시저 등 설정)
		*/
		vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

		//	[사용할 그래픽 파이프 라인을 설정하는 명령 기록]
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);

		// 뷰포트 정보 입력
		VkViewport viewport{};
		viewport.x = 0.0f;									// 뷰포트의 시작 x 좌표
		viewport.y = 0.0f;									// 뷰포트의 시작 y 좌표
		viewport.width = (float) swapChainExtent.width;		// 뷰포트의 width 크기
		viewport.height = (float) swapChainExtent.height;	// 뷰포트의 height 크기
		viewport.minDepth = 0.0f;							// 뷰포트의 최소 깊이
		viewport.maxDepth = 1.0f;							// 뷰포트의 최대 깊이
		vkCmdSetViewport(commandBuffer, 0, 1, &viewport);	// [커맨드 버퍼에 뷰포트 설정 등록]

		// 시저 정보 입력
		VkRect2D scissor{};
		scissor.offset = {0, 0};							// 시저의 시작 좌표
		scissor.extent = swapChainExtent;					// 시저의 width, height
		vkCmdSetScissor(commandBuffer, 0, 1, &scissor);		// [커맨드 버퍼에 시저 설정 등록]

		// 버텍스 정보 입력
		VkBuffer vertexBuffers[] = {vertexBuffer};			// 버택스 버퍼 객체
		VkDeviceSize offsets[] = {0};						// 버텍스 버퍼 메모리의 시작 위치 offset
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets); // 커맨드 버퍼에 버텍스 버퍼 바인딩

		// 인덱스 정보 입력
		vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32); // 커맨드 버퍼에 인덱스 버퍼 바인딩 (4번째 매개변수 index 데이터 타입 uint32 설정)

		// 디스크립터 셋을 커맨드 버퍼에 바인딩
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);

		// [Drawing 작업을 요청하는 명령 기록]
		vkCmdDrawIndexed(commandBuffer, static_cast<uint32_t>(indices.size()), 1, 0, 0, 0); // index로 drawing 하는 명령 기록

		/*
			[렌더 패스 종료]
			1. 자원의 정리 및 레이아웃 전환 (최종 작업을 위해 attachment에 정의된 finalLayout 설정)
			2. Load, Store 작업 (각 attachment에 정해진 load, store 작업 실행)
			3. 렌더 패스의 종료를 GPU에 알려 자원 재활용 등이 가능해짐
		*/ 
		vkCmdEndRenderPass(commandBuffer);

		// [커맨드 버퍼 기록 종료]
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
			throw std::runtime_error("failed to record command buffer!");
		}
	}

	/*
		[동기화 오브젝트 생성]
		세마포어 - GPU, GPU 작업간 동기화
		펜스 - CPU, GPU 작업간 동기화
	*/
	void createSyncObjects() {
		// 세마포어, 펜스 vector 동시에 처리할 최대 프레임 버퍼 수만큼 할당
		imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
		renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
		inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

		// 세마포어 생성 설정 값 준비
		VkSemaphoreCreateInfo semaphoreInfo{};
		semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

		// 펜스 생성 설정 값 준비
		VkFenceCreateInfo fenceInfo{};
		fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;         // signal 등록된 상태로 생성 (시작하자마자 wait으로 시작하므로 필요한 FLAG)

		// 세마포어, 펜스 생성
		for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
			if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
				vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
				vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS) {
				throw std::runtime_error("failed to create synchronization objects for a frame!");
			}
		}
	}

	// Uniform 변수에 해당하는 값을 구한 후 매핑된 GPU 메모리에 복사
    void updateUniformBuffer(uint32_t currentImage) {
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
        memcpy(uniformBuffersMapped[currentImage], &ubo, sizeof(ubo));
    }

	/*
		[다중 Frame 방식으로 그리기]
		동시에 작업 가능한 최대 Frame 개수만큼 자원을 생성하여 사용 (semaphore, fence, commandBuffer)
		작업할 Frame에 대한 커맨드 버퍼에 명령을 기록하여 GPU에 작업들을(렌더링 + 프레젠테이션) 맡기고 다음 Frame을 Draw 하러 이동
		Frame 작업을 병렬로 실행 (최대 Frame 개수의 작업이 진행 중이면 다음 작업은 Fence의 signal을 기다리며 대기)
	*/
	void drawFrame() {
		// [이전 GPU 작업 대기]
		// 동시에 작업 가능한 최대 Frame 개수만큼 작업 중인 경우 대기 (가장 먼저 시작한 Frame 작업이 끝나서 Fence에 signal을 보내기를 기다림)
		vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);
 
		// [작업할 image 준비]
		// 이번 Frame 에서 사용할 이미지 준비 및 해당 이미지 index 받아오기 (준비가 끝나면 signal 보낼 세마포어 등록)
		// vkAcquireNextImageKHR 함수는 CPU에서 swapChain과 surface의 호환성을 확인하고 GPU에 이미지 준비 명령을 내리는 함수
		// 만약 image가 프레젠테이션 큐에 작업이 진행 중이거나 대기 중이면 해당 image는 사용하지 않고 대기한다.
		uint32_t imageIndex;
		VkResult result = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);

		// image 준비 실패로 인한 오류 처리
		if (result == VK_ERROR_OUT_OF_DATE_KHR) {
			// 스왑 체인이 surface 크기와 호환되지 않는 경우로(창 크기 변경), 스왑 체인 재생성 후 다시 draw
			swapChainManager->recreateSwapChain(window, deviceManager.get(), surface, renderPass);
			return;
		} else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
			// 진짜 오류 gg
			throw std::runtime_error("failed to acquire swap chain image!");
		}

		// Uniform buffer 업데이트
		updateUniformBuffer(currentFrame);

		// [Fence 초기화]
		// Fence signal 상태 not signaled 로 초기화
		vkResetFences(device, 1, &inFlightFences[currentFrame]);

		// [Command Buffer에 명령 기록]
		// 커맨드 버퍼 초기화 및 명령 기록
		vkResetCommandBuffer(commandBuffers[currentFrame], /*VkCommandBufferResetFlagBits*/ 0); // 두 번째 매개변수인 Flag 를 0으로 초기화하면 기본 초기화 진행
		recordCommandBuffer(commandBuffers[currentFrame], imageIndex); // 현재 작업할 image의 index와 commandBuffer를 전송

		// [렌더링 Command Buffer 제출]
		// 렌더링 커맨드 버퍼 제출 정보 객체 생성
		VkSubmitInfo submitInfo{};
		submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

		// 작업 실행 신호를 받을 대기 세마포어 설정 (해당 세마포어가 signal 상태가 되기 전엔 대기)
		VkSemaphore waitSemaphores[] = {imageAvailableSemaphores[currentFrame]};				
		VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT}; 	
		submitInfo.waitSemaphoreCount = 1;														// 대기 세마포어 개수
		submitInfo.pWaitSemaphores = waitSemaphores;											// 대기 세마포어 등록
		submitInfo.pWaitDstStageMask = waitStages;												// 대기할 시점 등록 (그 전까지는 세마포어 상관없이 그냥 진행)	

		// 커맨드 버퍼 등록
		submitInfo.commandBufferCount = 1;														// 커맨드 버퍼 개수 등록
		submitInfo.pCommandBuffers = &commandBuffers[currentFrame];								// 커매드 버퍼 등록

		// 작업이 완료된 후 신호를 보낼 세마포어 설정 (작업이 끝나면 해당 세마포어 signal 상태로 변경)
		VkSemaphore signalSemaphores[] = {renderFinishedSemaphores[currentFrame]};
		submitInfo.signalSemaphoreCount = 1;													// 작업 끝나고 신호를 보낼 세마포어 개수
		submitInfo.pSignalSemaphores = signalSemaphores;										// 작업 끝나고 신호를 보낼 세마포어 등록

		// 커맨드 버퍼 제출
		if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS) {
			throw std::runtime_error("failed to submit draw command buffer!");
		}

		// [프레젠테이션 Command Buffer 제출]
		// 프레젠테이션 커맨드 버퍼 제출 정보 객체 생성
		VkPresentInfoKHR presentInfo{};
		presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

		// 작업 실행 신호를 받을 대기 세마포어 설정
		presentInfo.waitSemaphoreCount = 1;														// 대기 세마포어 개수
		presentInfo.pWaitSemaphores = signalSemaphores;											// 대기 세마포어 등록

		// 제출할 스왑 체인 설정
		VkSwapchainKHR swapChains[] = {swapChain};
		presentInfo.swapchainCount = 1;															// 스왑체인 개수
		presentInfo.pSwapchains = swapChains;													// 스왑체인 등록
		presentInfo.pImageIndices = &imageIndex;												// 스왑체인에서 표시할 이미지 핸들 등록

		// 프레젠테이션 큐에 이미지 제출
		result = vkQueuePresentKHR(presentQueue, &presentInfo);

		// 프레젠테이션 실패 오류 발생 시
		if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized) {
			// 스왑 체인 크기와 surface의 크기가 호환되지 않는 경우
			framebufferResized = false;
			swapChainManager->recreateSwapChain(window, deviceManager.get(), surface, renderPass); // 변경된 surface에 맞는 SwapChain, ImageView, FrameBuffer 생성 
		} else if (result != VK_SUCCESS) {
			// 진짜 오류 gg
			throw std::runtime_error("failed to present swap chain image!");
		}

		// [프레임 인덱스 증가]
		// 다음 작업할 프레임 변경
		currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
	}

	/*
		[이미지 객체 생성 및 메모리 할당]
		1. 이미지 객체 생성
		2. 이미지 객체가 사용할 메모리 할당
		3. 이미지 객체에 할당한 메모리 바인딩
	*/
	void createImage(uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory) {
		// 이미지 객체를 만드는데 사용되는 구조체
		VkImageCreateInfo imageInfo{};
		imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageInfo.imageType = VK_IMAGE_TYPE_2D;					// 이미지의 차원을 설정
		imageInfo.extent.width = width;							// 이미지의 너비 지정
		imageInfo.extent.height = height;						// 이미지의 높이 지정 
		imageInfo.extent.depth = 1;								// 이미지의 깊이 지정 (2D 이미지의 경우 depth는 1로 지정해야 함)
		imageInfo.mipLevels = mipLevels;						// 생성할 mipLevel의 개수 지정
		imageInfo.arrayLayers = 1;								// 생성할 이미지 레이어 수 (큐브맵의 경우 6개 생성)
		imageInfo.format = format;								// 이미지의 포맷을 지정하며, 채널 구성과 각 채널의 비트 수를 정의
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
		allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);		// 메모리 유형과 속성 설정

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
	uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
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
	VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels) {
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
};

int main() {
	App app;

	try {
		app.run();
	} catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
