#include "app.h"

void App::run()
{
	initWindow();
	initVulkan();
	mainLoop();
	cleanup();
}

// glfw 실행, window 생성, 콜백 함수 등록
void App::initWindow()
{
	glfwInit();

	// glfw로 창관리만 하고 렌더링 API는 안 쓴다는 설정
	// 렌더링 및 그래픽 처리는 외부 API인 Vulkan으로 함
	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Vulkan", nullptr, nullptr);

	// window에 현재 App 객체를 바인딩
	glfwSetWindowUserPointer(window, this);

	// 콜백 함수 등록
	glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);
	glfwSetKeyCallback(window, keyEventCallback);
	glfwSetCursorPosCallback(window, cursorPosCallback);
	glfwSetMouseButtonCallback(window, mouseButtonCallback);
}

// callback 함수 start

void App::framebufferResizeCallback(GLFWwindow *window, int width, int height)
{
	// window에 바인딩된 객체 호출 및 framebufferResized = true 설정
	App *app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
	app->framebufferResized = true;
}

void App::cursorPosCallback(GLFWwindow *window, double x, double y)
{
	App *app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
	app->mouseMove(x, y);
}

void App::mouseButtonCallback(GLFWwindow *window, int button, int action, int modifier)
{
	App *app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
	double x, y;
	glfwGetCursorPos(window, &x, &y);
	app->mouseButton(button, action, x, y);
}

void App::keyEventCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, true);
	}
}

// camera control process start

void App::mouseButton(int button, int action, double x, double y)
{
	if (button == GLFW_MOUSE_BUTTON_RIGHT)
	{
		if (action == GLFW_PRESS)
		{
			camera.saveCurrentPos((float)x, (float)y);
			cameraControl = true;
		}
		else if (action == GLFW_RELEASE)
			cameraControl = false;
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (action == GLFW_PRESS)
		{
			shootControl = true;
		}
		else if (action == GLFW_RELEASE)
		{
			shootControl = false;
			canShoot = true;
		}
	}
}

void App::mouseMove(double x, double y)
{
	if (!cameraControl)
	{
		return;
	}
	glm::vec2 pos((float)x, (float)y);
	camera.rotate(pos);
}

void App::processCameraControl()
{
	if (!cameraControl)
	{
		return;
	}

	bool pressW = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
	bool pressS = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
	bool pressD = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
	bool pressA = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
	bool pressE = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
	bool pressQ = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;

	camera.move(pressW, pressS, pressD, pressA, pressE, pressQ);
}

void App::processEvents()
{
	if (shootControl && canShoot && shootNum < SHOOT_MAX)
	{
		glm::vec3 cameraPos = camera.getCameraPos();
		glm::vec3 cameraFront = camera.getCameraFront();

		ale::Transform sphereXf(cameraPos, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
		models.push_back(
			Model::createSphere(deviceManager.get(), commandManager->getCommandPool(), sphereXf, "models/sphere.png"));
		transforms.push_back(sphereXf);

		int32_t idx = models.size() - 1;
		world->createBody(models[idx], idx);
		world->registerBodyForce(idx, cameraFront * 100000.0f);

		models[idx]->createDescriptorSets(device, descriptorPool->get(), renderer->getDescriptorSetLayout());

		canShoot = false;
		shootNum++;
	}
}

// vulkan init start
void App::initVulkan()
{
	// instance 생성
	vulkanInstance = VulkanInstance::create(window);

	// deviceManager 생성
	deviceManager = DeviceManager::create(vulkanInstance.get());
	device = deviceManager->getLogicalDevice();

	// swapChainManager 생성
	swapChainManager = SwapChainManager::create(window, deviceManager.get(), vulkanInstance->getSurface());

	// renderer 생성
	renderer = Renderer::create(deviceManager.get(), swapChainManager->getSwapChainImageFormat());

	// commandPool 생성
	commandManager = CommandManager::create(deviceManager.get());

	// Framebuffer 생성
	swapChainManager->createFramebuffers(deviceManager.get(), renderer->getRenderPass());

	// model 생성
	createModels();

	// physics world 생성
	createWorld();

	// descriptorPool 생성
	descriptorPool = DescriptorPool::create(device, models, SHOOT_MAX);

	// descriptorSets 생성
	for (std::unique_ptr<Model> &model : models)
	{
		model->createDescriptorSets(device, descriptorPool->get(), renderer->getDescriptorSetLayout());
	}

	// 동기화 도구 생성
	syncObject = SyncObject::create(device);
}

/*
	렌더링 루프 실행
*/
void App::mainLoop()
{
	// // 힘 등록
	// world->registerBodyForce(0, glm::vec3(500.0f, 0.0f, 0.0f));

	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();
		processCameraControl();
		processEvents();
		// calculate positions
		world->startFrame();
		world->runPhysics();
		drawFrame();
	}
	vkDeviceWaitIdle(device); // 종료시 실행 중인 GPU 작업을 전부 기다림
}

/*
	[사용한 자원들 정리]
*/
void App::cleanup()
{
	for (std::unique_ptr<Model> &model : models)
	{
		model->clear();
	}
	swapChainManager->cleanupSwapChain();
	renderer->clear();
	descriptorPool->clear();
	syncObject->clear();
	commandManager->clear();
	deviceManager->clear();
	vulkanInstance->clear();
	glfwDestroyWindow(window);
	glfwTerminate();
}

// rendering start
/*
	[다중 Frame 방식으로 그리기]
	동시에 작업 가능한 최대 Frame 개수만큼 자원을 생성하여 사용 (semaphore, fence, commandBuffer)
	작업할 Frame에 대한 커맨드 버퍼에 명령을 기록하여 GPU에 작업들을(렌더링 + 프레젠테이션) 맡기고 다음 Frame을 Draw
   하러 이동 Frame 작업을 병렬로 실행 (최대 Frame 개수의 작업이 진행 중이면 다음 작업은 Fence의 signal을 기다리며 대기)
*/
void App::drawFrame()
{

	// 이미지 준비
	uint32_t imageIndex;
	if (!tryPrepareImage(&imageIndex))
	{
		return;
	}

	// 유니폼 버퍼 업데이트
	updateUniformBuffer();

	// Command Buffer에 명령 기록
	recordCommandBuffer(commandManager->getCommandBuffer(currentFrame), imageIndex);

	// Rendering Command Buffer 제출
	submitRenderingCommandBuffer();

	// Presentation Command Buffer 제출
	submitPresentationCommandBuffer(imageIndex);

	// [프레임 인덱스 증가]
	// 다음 작업할 프레임 변경
	currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void App::createModels()
{
	ale::Transform groundXf(glm::vec3(0.0f, -0.51f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	models.push_back(
		Model::createGround(deviceManager.get(), commandManager->getCommandPool(), groundXf, "models/Greyground.jpg"));
	transforms.push_back(groundXf);

	// 박스
	int32_t N = 5;
	float bz = 0.0f;
	for (int32_t i = 0; i < N; i++)
	{
		float by = 0.0f;
		for (int32_t j = 0; j < N; j++)
		{
			float bx = 0.0f;
			for (int32_t k = 0; k < N; k++)
			{
				ale::Transform boxXf(glm::vec3(bx, by, bz),
									 glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
				models.push_back(Model::createBox(deviceManager.get(), commandManager->getCommandPool(), boxXf,
												  "models/container.png"));
				transforms.push_back(boxXf);
				bx = bx + 1.0f;
			}
			by = by + 1.0f;
		}
		bz = bz + 1.0f;
	}

	// 실린더
	// int32_t M = 2;
	// float cz = 0.0f;
	// for (int32_t i = 0; i < M; i++)
	// {
	// 	float cy = 0.0f;
	// 	for (int32_t j = 0; j < M; j++)
	// 	{
	// 		float cx = 0.0f;
	// 		for (int32_t k = 0; k < M; k++)
	// 		{
	// 			ale::Transform cylinderXf(
	// 				glm::vec3(cx, cy, cz),
	// 				glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// 			models.push_back(Model::createCylinder(deviceManager.get(), commandManager->getCommandPool(),
	// 												   cylinderXf, "models/container.png"));
	// 			transforms.push_back(cylinderXf);
	// 			cx = cx + 1.0f;
	// 		}
	// 		cy = cy + 1.0f;
	// 	}
	// 	cz = cz + 1.0f;
	// }

	// 구
	// int32_t L = 2;
	// float sz = 0.0f;
	// for (int32_t i = 0; i < L; i++)
	// {
	// 	float sy = 0.0f;
	// 	for (int32_t j = 0; j < L; j++)
	// 	{
	// 		float sx = 0.0f;
	// 		for (int32_t k = 0; k < L; k++)
	// 		{
	// 			ale::Transform sphereXf(
	// 				glm::vec3(sx, sy, sz),
	// 				glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// 			models.push_back(Model::createSphere(deviceManager.get(), commandManager->getCommandPool(), sphereXf,
	// 												 "models/sphere.png"));
	// 			transforms.push_back(sphereXf);
	// 			sx = sx + 1.0f;
	// 		}
	// 		sy = sy + 1.0f;
	// 	}
	// 	sz = sz + 1.0f;
	// }

	// 캡슐
	// int32_t O = 2;
	// float cz = 0.0f;
	// for (int32_t i = 0; i < 1; i++)
	// {
	// 	float cy = 0.5f;
	// 	for (int32_t j = 0; j < O; j++)
	// 	{
	// 		float cx = 0.0f;
	// 		for (int32_t k = 0; k < O; k++)
	// 		{
	// 			ale::Transform capsuleXF(
	// 				glm::vec3(cx, cy, cz),
	// 				glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// 			models.push_back(Model::createCapsule(deviceManager.get(), commandManager->getCommandPool(), capsuleXF,
	// 												  "models/container.png"));
	// 			transforms.push_back(capsuleXF);
	// 			cx = cx + 1.0f;
	// 		}
	// 		cy = cy + 2.0f;
	// 	}
	// 	cz = cz + 1.0f;
	// }

	// ale::Transform capsuleXF(glm::vec3(0.0f, 6.5f, 0.0f),
	// 						 glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// models.push_back(
	// 	Model::createCapsule(deviceManager.get(), commandManager->getCommandPool(), capsuleXF, "models/container.png"));
	// transforms.push_back(capsuleXF);

	// ale::Transform capsuleXF0(glm::vec3(1.0f, 6.5f, 0.0f),
	// 						 glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// models.push_back(
	// 	Model::createCapsule(deviceManager.get(), commandManager->getCommandPool(), capsuleXF0,
	// "models/container.png")); transforms.push_back(capsuleXF0);

	// ale::Transform cylinderXF(glm::vec3(0.0f, 2.5f, 0.0f),
	// 						 glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// models.push_back(
	// 	Model::createCylinder(deviceManager.get(), commandManager->getCommandPool(), cylinderXF,
	// "models/container.png")); transforms.push_back(cylinderXF);

	// ale::Transform cylinderXF1(glm::vec3(1.0f, 0.5f, 0.0f),
	// 						   glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// models.push_back(Model::createCylinder(deviceManager.get(), commandManager->getCommandPool(), cylinderXF1,
	// 									   "models/container.png"));
	// transforms.push_back(cylinderXF1);

	// ale::Transform boxXF(glm::vec3(0.0f, 0.5f, 0.0f),
	// 					 glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// models.push_back(
	// 	Model::createBox(deviceManager.get(), commandManager->getCommandPool(), boxXF, "models/container.png"));
	// transforms.push_back(boxXF);

	// ale::Transform boxXF1(glm::vec3(1.0f, 2.5f, 0.0f),
	// 					  glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f))));
	// models.push_back(
	// 	Model::createBox(deviceManager.get(), commandManager->getCommandPool(), boxXF1, "models/container.png"));
	// transforms.push_back(boxXF1);
}

void App::createWorld()
{
	world = new ale::World(static_cast<uint32_t>(models.size()), *this);

	// std::cout << "App::Create World\n";
	for (size_t i = 0; i < models.size(); ++i)
	{
		world->createBody(models[i], static_cast<int32_t>(i));
	}
	// std::cout << "App::Create World end\n";
}

bool App::tryPrepareImage(uint32_t *imageIndex)
{
	// [이전 GPU 작업 대기]
	// 동시에 작업 가능한 최대 Frame 개수만큼 작업 중인 경우 대기 (가장 먼저 시작한 Frame 작업이 끝나서 Fence에 signal을
	// 보내기를 기다림)
	vkWaitForFences(device, 1, syncObject->getInFlightFencePointer(currentFrame), VK_TRUE, UINT64_MAX);

	// [작업할 image 준비]
	// 이번 Frame 에서 사용할 이미지 준비 및 해당 이미지 index 받아오기 (준비가 끝나면 signal 보낼 세마포어 등록)
	// vkAcquireNextImageKHR 함수는 CPU에서 swapChain과 surface의 호환성을 확인하고 GPU에 이미지 준비 명령을 내리는 함수
	// 만약 image가 프레젠테이션 큐에 작업이 진행 중이거나 대기 중이면 해당 image는 사용하지 않고 대기한다.
	VkResult result =
		vkAcquireNextImageKHR(device, swapChainManager->getSwapChain(), UINT64_MAX,
							  syncObject->getImageAvailableSemaphore(currentFrame), VK_NULL_HANDLE, imageIndex);

	// image 준비 실패로 인한 오류 처리
	if (result == VK_ERROR_OUT_OF_DATE_KHR)
	{
		// 스왑 체인이 surface 크기와 호환되지 않는 경우로(창 크기 변경), 스왑 체인 재생성 후 다시 draw
		swapChainManager->recreateSwapChain(window, deviceManager.get(), vulkanInstance->getSurface(),
											renderer->getRenderPass());
		return false;
	}
	else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
	{
		// 진짜 오류 gg
		throw std::runtime_error("failed to acquire swap chain image!");
	}

	// [Fence 초기화]
	// Fence signal 상태 not signaled 로 초기화
	vkResetFences(device, 1, syncObject->getInFlightFencePointer(currentFrame));

	return true;
}

void App::updateUniformBuffer()
{
	VkExtent2D extent = swapChainManager->getSwapChainExtent();

	UniformBufferObject ubo;
	ubo.model = glm::mat4(1.0f);
	ubo.view = camera.getViewMatrix();
	ubo.proj = glm::perspective(glm::radians(45.0f), (float)extent.width / (float)extent.height, 0.01f, 100.0f);
	ubo.proj[1][1] *= -1;

	// Uniform buffer 업데이트
	// for (std::unique_ptr<Model> &model : models)
	// {
	// 	// glm::mat4 transform = glm::mat4(1.0f);
	// 	// glm::quat orientation = model->getBody()->getTransform().orientation;
	// 	// glm::vec3 position = model->getBody()->getTransform().position;

	// 	// glm::mat3 rotationMatrix = glm::toMat3(orientation);

	// 	// transform = glm::mat4(rotationMatrix);

	// 	// transform[3] = glm::vec4(position, 1.0f);
	// 	ubo.model = calculateTransformMatrix();
	// 	model->updateUniformBuffer(ubo, currentFrame);
	// }

	for (size_t i = 0; i < models.size(); ++i)
	{
		glm::mat4 transform;
		calculateTransformMatrix(static_cast<int32_t>(i), transform);
		ubo.model = transform;
		models[i]->updateUniformBuffer(ubo, currentFrame);
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
void App::recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex)
{
	// 커맨드 버퍼 초기화
	vkResetCommandBuffer(
		commandBuffer,
		/*VkCommandBufferResetFlagBits*/ 0); // 두 번째 매개변수인 Flag 를 0으로 초기화하면 기본 초기화 진행

	// 커맨드 버퍼 기록을 위한 정보 객체
	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

	// GPU에 필요한 작업을 모두 커맨드 버퍼에 기록하기 시작
	if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
	{
		throw std::runtime_error("failed to begin recording command buffer!");
	}

	// 렌더 패스 정보 지정
	VkRenderPassBeginInfo renderPassInfo{};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassInfo.renderPass = renderer->getRenderPass();						  // 렌더 패스 등록
	renderPassInfo.framebuffer = swapChainManager->getFramebuffers()[imageIndex]; // 프레임 버퍼 등록
	renderPassInfo.renderArea.offset = {0, 0};									  // 렌더링 시작 좌표 등록
	renderPassInfo.renderArea.extent =
		swapChainManager
			->getSwapChainExtent(); // 렌더링 width, height 등록 (보통 프레임버퍼, 스왑체인의 크기와 같게 설정)

	std::array<VkClearValue, 2> clearValues{};
	clearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
	clearValues[1].depthStencil = {1.0f, 0};

	renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size()); // clear color 개수 등록
	renderPassInfo.pClearValues = clearValues.data(); // clear color 등록 (첨부한 attachment 개수와 같게 등록)

	/*
		[렌더 패스를 시작하는 명령을 기록]
		GPU에서 렌더링에 필요한 자원과 설정을 준비 (대략 과정)
		1. 렌더링 자원 초기화 (프레임 버퍼와 렌더 패스에 등록된 attachment layout 초기화)
		2. 서브패스 및 attachment 설정 적용
		3. 렌더링 작업을 위한 컨텍스트 준비 (뷰포트, 시저 등 설정)
	*/
	vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

	//	[사용할 그래픽 파이프 라인을 설정하는 명령 기록]
	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, renderer->getGraphicsPipeline());

	VkExtent2D swapChainExtent = swapChainManager->getSwapChainExtent();

	// 뷰포트 정보 입력
	VkViewport viewport{};
	viewport.x = 0.0f;								  // 뷰포트의 시작 x 좌표
	viewport.y = 0.0f;								  // 뷰포트의 시작 y 좌표
	viewport.width = (float)swapChainExtent.width;	  // 뷰포트의 width 크기
	viewport.height = (float)swapChainExtent.height;  // 뷰포트의 height 크기
	viewport.minDepth = 0.0f;						  // 뷰포트의 최소 깊이
	viewport.maxDepth = 1.0f;						  // 뷰포트의 최대 깊이
	vkCmdSetViewport(commandBuffer, 0, 1, &viewport); // [커맨드 버퍼에 뷰포트 설정 등록]

	// 시저 정보 입력
	VkRect2D scissor{};
	scissor.offset = {0, 0};						// 시저의 시작 좌표
	scissor.extent = swapChainExtent;				// 시저의 width, height
	vkCmdSetScissor(commandBuffer, 0, 1, &scissor); // [커맨드 버퍼에 시저 설정 등록]

	for (std::unique_ptr<Model> &model : models)
	{
		model->recordDrawCommand(commandBuffer, renderer->getPipelineLayout(), currentFrame);
	}

	/*
		[렌더 패스 종료]
		1. 자원의 정리 및 레이아웃 전환 (최종 작업을 위해 attachment에 정의된 finalLayout 설정)
		2. Load, Store 작업 (각 attachment에 정해진 load, store 작업 실행)
		3. 렌더 패스의 종료를 GPU에 알려 자원 재활용 등이 가능해짐
	*/
	vkCmdEndRenderPass(commandBuffer);

	// [커맨드 버퍼 기록 종료]
	if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
	{
		throw std::runtime_error("failed to record command buffer!");
	}
}

void App::submitRenderingCommandBuffer()
{
	// [렌더링 Command Buffer 제출]
	// 렌더링 커맨드 버퍼 제출 정보 객체 생성
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

	// 작업 실행 신호를 받을 대기 세마포어 설정 (해당 세마포어가 signal 상태가 되기 전엔 대기)
	VkSemaphore waitSemaphores[] = {syncObject->getImageAvailableSemaphore(currentFrame)};
	VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
	submitInfo.waitSemaphoreCount = 1;			 // 대기 세마포어 개수
	submitInfo.pWaitSemaphores = waitSemaphores; // 대기 세마포어 등록
	submitInfo.pWaitDstStageMask = waitStages; // 대기할 시점 등록 (그 전까지는 세마포어 상관없이 그냥 진행)

	// 커맨드 버퍼 등록
	submitInfo.commandBufferCount = 1; // 커맨드 버퍼 개수 등록
	submitInfo.pCommandBuffers = commandManager->getCommandBufferPointer(currentFrame); // 커매드 버퍼 등록

	// 작업이 완료된 후 신호를 보낼 세마포어 설정 (작업이 끝나면 해당 세마포어 signal 상태로 변경)
	VkSemaphore signalSemaphores[] = {syncObject->getRenderFinishedSemaphore(currentFrame)};
	submitInfo.signalSemaphoreCount = 1;			 // 작업 끝나고 신호를 보낼 세마포어 개수
	submitInfo.pSignalSemaphores = signalSemaphores; // 작업 끝나고 신호를 보낼 세마포어 등록

	// 커맨드 버퍼 제출
	if (vkQueueSubmit(deviceManager->getGraphicsQueue(), 1, &submitInfo, syncObject->getInFlightFence(currentFrame)) !=
		VK_SUCCESS)
	{
		throw std::runtime_error("failed to submit draw command buffer!");
	}
}

void App::submitPresentationCommandBuffer(uint32_t imageIndex)
{
	// 프레젠테이션 커맨드 버퍼 제출 정보 객체 생성
	VkPresentInfoKHR presentInfo{};
	presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

	// 작업 실행 신호를 받을 대기 세마포어 설정
	VkSemaphore signalSemaphores[] = {syncObject->getRenderFinishedSemaphore(currentFrame)};
	presentInfo.waitSemaphoreCount = 1;				// 대기 세마포어 개수
	presentInfo.pWaitSemaphores = signalSemaphores; // 대기 세마포어 등록

	// 제출할 스왑 체인 설정
	VkSwapchainKHR swapChains[] = {swapChainManager->getSwapChain()};
	presentInfo.swapchainCount = 1;			 // 스왑체인 개수
	presentInfo.pSwapchains = swapChains;	 // 스왑체인 등록
	presentInfo.pImageIndices = &imageIndex; // 스왑체인에서 표시할 이미지 핸들 등록

	// 프레젠테이션 큐에 이미지 제출
	VkResult result = vkQueuePresentKHR(deviceManager->getPresentQueue(), &presentInfo);

	// 프레젠테이션 실패 오류 발생 시
	if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized)
	{
		// 스왑 체인 크기와 surface의 크기가 호환되지 않는 경우
		framebufferResized = false;
		swapChainManager->recreateSwapChain(
			window, deviceManager.get(), vulkanInstance->getSurface(),
			renderer->getRenderPass()); // 변경된 surface에 맞는 SwapChain, ImageView, FrameBuffer 생성
	}
	else if (result != VK_SUCCESS)
	{
		// 진짜 오류 gg
		throw std::runtime_error("failed to present swap chain image!");
	}
}

const ale::Transform &App::getTransformById(int32_t xfId) const
{
	// check xfId range
	return transforms[xfId];
}

void App::setTransformById(int32_t xfId, const ale::Transform &xf)
{
	// check xfId range
	transforms[xfId] = xf;
}

void App::calculateTransformMatrix(int32_t xfId, glm::mat4 &transformMatrix)
{
	ale::Transform xf = transforms[xfId];

	glm::mat4 rotationMatrix = glm::toMat4(glm::normalize(xf.orientation));
	glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), xf.position);

	transformMatrix = translationMatrix * rotationMatrix;
}
