#include "../include/common.h"
#include "../include/vulkanInstance.h"
#include "../include/deviceManager.h"
#include "../include/swapChainManager.h"
#include "../include/image.h"
#include "../include/renderer.h"
#include "../include/buffer.h"
#include "../include/mesh.h"

#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <limits>


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
		VkPipelineLayout pipelineLayout;
		VkPipeline graphicsPipeline;

	VkCommandPool commandPool;
	

	std::vector<std::unique_ptr<Mesh>> meshes;
	std::vector<std::unique_ptr<Material>> materials;
		// std::vector<Vertex> vertices;
		// std::vector<uint32_t> indices;
		// std::unique_ptr<VertexBuffer> vertexBuffer;
		// std::unique_ptr<IndexBuffer> indexBuffer;
		// std::unique_ptr<UniformBuffer> uniformBuffer;
		// std::unique_ptr<TextureImage> texture;
		// 	std::vector<VkBuffer> uniformBuffers;
		// 	std::vector<VkDeviceMemory> uniformBuffersMemory;
		// 	std::vector<void*> uniformBuffersMapped;


	VkDescriptorPool descriptorPool;
	
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
			pipelineLayout = renderer->getPipelineLayout();
			graphicsPipeline = renderer->getGraphicsPipeline();
			createCommandBuffers();


		swapChainManager->createFramebuffers(deviceManager.get(), renderPass);
		swapChainFramebuffers = swapChainManager->getFramebuffers();

		// Model 클래스
		loadModel("models/backpack/backpack.obj");	

		// Descriptor 클래스		
		createDescriptorPool();

		for (std::unique_ptr<Mesh>& mesh : meshes)
		{
			mesh->createDescriptorSets(device, descriptorPool, renderer->getDescriptorSetLayout());
		}

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
		swapChainManager->cleanupSwapChain();
		renderer->clear();

		for (std::unique_ptr<Mesh>& mesh : meshes) { mesh->clear(); }
		for (std::unique_ptr<Material>& material : materials) { material->clear(); }
		
		vkDestroyDescriptorPool(device, descriptorPool, nullptr);			// 디스크립터 풀 삭제
		// 세마포어, 펜스 파괴
		for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
			vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
			vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
			vkDestroyFence(device, inFlightFences[i], nullptr);
		}
		vkDestroyCommandPool(device, commandPool, nullptr); 	  	// 커맨드 풀 파괴
		deviceManager->clear();
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

	// .obj 파일을 읽고 vertices, indices 채우기
	void loadModel(std::string filename) {
		Assimp::Importer importer;
		// scene 구조체 받아오기
		const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_FlipUVs);
		std::string dirname = filename.substr(0, filename.find_last_of("/"));
		
		// scene load 오류 처리
		if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
		{
			throw std::runtime_error("failed to load obj file!");
		}

		// scene 안에있는 material 개수만큼 반복
		for (uint32_t i = 0; i < scene->mNumMaterials; i++)
		{
			// scene의 i번째 material 정보 get
			aiMaterial* materialInfo = scene->mMaterials[i];
			materials.push_back(Material::create(materialInfo, deviceManager.get(), commandPool, dirname));
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
			uint32_t meshIndex = node->mMeshes[i];
			aiMesh* mesh = scene->mMeshes[meshIndex];
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
		std::vector<Vertex> vertices;
		vertices.resize(mesh->mNumVertices);
		for (uint32_t i = 0; i < mesh->mNumVertices; i++)
		{
			Vertex& v = vertices[i];
			v.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
			v.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
			v.texCoord = glm::vec2(mesh->mTextureCoords[0][i].x, -mesh->mTextureCoords[0][i].y);
		}

		std::vector<uint32_t> indices;
		indices.resize(mesh->mNumFaces * 3);
		// face의 개수 = triangle 개수
		for (uint32_t i = 0; i < mesh->mNumFaces; i++)
		{
			indices[3 * i] = mesh->mFaces[i].mIndices[0];
			indices[3 * i + 1] = mesh->mFaces[i].mIndices[1];
			indices[3 * i + 2] = mesh->mFaces[i].mIndices[2];
		}

		std::unique_ptr<Mesh> newMesh = Mesh::create(deviceManager.get(), commandPool, vertices, indices);
		// mesh의 mMaterialINdex가 0이상이면 이 mesh는 material을 갖고 있으므로
		// 해당 material값을 setting 해준다. 
		if (mesh->mMaterialIndex >= 0)
			newMesh->setMaterial(materials[mesh->mMaterialIndex].get());
		meshes.push_back(std::move(newMesh));
	}

	// 디스크립터 풀 생성
	void createDescriptorPool() {
		
		// 디스크립터 풀의 타입별 디스크립터 개수를 설정하는 구조체
        std::array<VkDescriptorPoolSize, 2> poolSizes{};
        poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;											// 유니폼 버퍼 설정
        poolSizes[0].descriptorCount = static_cast<uint32_t>(meshes.size() * MAX_FRAMES_IN_FLIGHT);						// 유니폼 버퍼 디스크립터 최대 개수 설정
        poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;									// 샘플러 설정
        poolSizes[1].descriptorCount = static_cast<uint32_t>(2 * meshes.size() * MAX_FRAMES_IN_FLIGHT);		// 샘플러 디스크립터 최대 개수 설정

		// 디스크립터 풀을 생성할 때 필요한 설정 정보를 담는 구조체
		VkDescriptorPoolCreateInfo poolInfo{};
		poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());			// 디스크립터 poolSize 구조체 개수
        poolInfo.pPoolSizes = poolSizes.data();										// 디스크립터 poolSize 구조체 배열
		poolInfo.maxSets = static_cast<uint32_t>(meshes.size() * MAX_FRAMES_IN_FLIGHT);				// 풀에 존재할 수 있는 총 디스크립터 셋 개수

		// 디스크립터 풀 생성
		if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
			throw std::runtime_error("failed to create descriptor pool!");
		}
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

		for (std::unique_ptr<Mesh>& mesh : meshes)
		{
			// 버텍스 정보 입력
			VkDeviceSize offsets[] = {0};						// 버텍스 버퍼 메모리의 시작 위치 offset
			vkCmdBindVertexBuffers(commandBuffer, 0, 1, mesh->getVertexBuffer()->getBuffers().data(), offsets); // 커맨드 버퍼에 버텍스 버퍼 바인딩

			// 인덱스 정보 입력
			vkCmdBindIndexBuffer(commandBuffer, mesh->getIndexBuffer()->getBuffer(), 0, VK_INDEX_TYPE_UINT32); // 커맨드 버퍼에 인덱스 버퍼 바인딩 (4번째 매개변수 index 데이터 타입 uint32 설정)

			// 디스크립터 셋을 커맨드 버퍼에 바인딩
			vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, mesh->getDescriptor(currentFrame), 0, nullptr);

			// [Drawing 작업을 요청하는 명령 기록]
			vkCmdDrawIndexed(commandBuffer, mesh->getIndexBuffer()->getIndicesSize(), 1, 0, 0, 0); // index로 drawing 하는 명령 기록

			/*
				[렌더 패스 종료]
				1. 자원의 정리 및 레이아웃 전환 (최종 작업을 위해 attachment에 정의된 finalLayout 설정)
				2. Load, Store 작업 (각 attachment에 정해진 load, store 작업 실행)
				3. 렌더 패스의 종료를 GPU에 알려 자원 재활용 등이 가능해짐
			*/ 
		}
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
		for (std::unique_ptr<Mesh>& mesh : meshes)
		{
			mesh->getUniformBuffer()->update(swapChainManager->getSwapChainExtent(), currentFrame);
		}

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
