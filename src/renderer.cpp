#include "../include/renderer.h"
#include "../include/buffer.h"

std::unique_ptr<Renderer> Renderer::create(DeviceManager* deviceManager, VkFormat swapChainImageFormat)
{
	std::unique_ptr<Renderer> renderer(new Renderer());
	renderer->init(deviceManager, swapChainImageFormat);
	return renderer;
}

void Renderer::clear()
{
	vkDestroyPipeline(device, graphicsPipeline, nullptr);      	// 파이프라인 객체 삭제
	vkDestroyPipelineLayout(device, pipelineLayout, nullptr);  	// 파이프라인 레이아웃 삭제
	vkDestroyRenderPass(device, renderPass, nullptr);         	// 렌더 패스 삭제
	vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);	// 디스크립터 셋 레이아수 삭제
}

void Renderer::init(DeviceManager* deviceManager, VkFormat swapChainImageFormat)
{
	device = deviceManager->getLogicalDevice();
	createRenderPass(deviceManager, swapChainImageFormat);
	createDescriptorSetLayout();
	createGraphicsPipeline(deviceManager->getMsaaSamples());
}

/*
	[렌더패스 생성]
	렌더패스 구성 요소
	1. attachment 설정
	2. subpass
*/
void Renderer::createRenderPass(DeviceManager* deviceManager, VkFormat swapChainImageFormat) {
	VkSampleCountFlagBits msaaSamples = deviceManager->getMsaaSamples();
	
	// [attachment 설정]
	// FrameBuffer의 attachment에 어떤 정보를 어떻게 기록할지 정하는 객체
	// 멀티 샘플링 color attachment 설정
	VkAttachmentDescription colorAttachment{};
	colorAttachment.format = swapChainImageFormat;		 					// 이미지 포맷 (스왑 체인과 일치 시킴)
	colorAttachment.samples = msaaSamples;			 						// 샘플 개수 (멀티 샘플링을 위한 값 사용)
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;					// 렌더링 전 버퍼 클리어 (렌더링 시작 시 기존 attachment의 데이터 처리 방법)
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE; 				// 렌더링 결과 저장 (렌더링 후 attachment를 메모리에 저장하는 방법 결정)
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE; 		// 이전 데이터 무시 (스텐실 버퍼의 loadOp)
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE; 		// 저장 x (스텐실 버퍼의 storeOp)
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; 				// 초기 레이아웃 설정을 UNDEFINED로 설정 (초기 데이터 가공을 하지 않기 때문에 가장 빠름)
	colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; // color attachment 레이아웃 설정 

	// depth attachment 설정
	VkAttachmentDescription depthAttachment{};
	depthAttachment.format = deviceManager->findDepthFormat();
	depthAttachment.samples = msaaSamples;
	depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;						// 초기 레이아웃 설정을 UNDEFINED로 설정
	depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL; // 최종 레이아웃 depth-stencil buffer로 사용

	// resolve attachment 설정
	// 멀티 샘플링 attachment를 단일 샘플링 attachment로 전환
	VkAttachmentDescription colorAttachmentResolve{};
	colorAttachmentResolve.format = swapChainImageFormat;
	colorAttachmentResolve.samples = VK_SAMPLE_COUNT_1_BIT;
	colorAttachmentResolve.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachmentResolve.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachmentResolve.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachmentResolve.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachmentResolve.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	colorAttachmentResolve.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

	// subpass가 attachment 설정 어떤 것을 어떻게 참조할지 정의
	// color attachment
	VkAttachmentReference colorAttachmentRef{};
	colorAttachmentRef.attachment = 0; 										// 특정 attachment 설정의 index
	colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;	// attachment 설정을 subpass 내에서
																			// 어떤 layout으로 쓸지 결정 (현재는 color attachment로 사용하는 설정)
	// depth attachment
	VkAttachmentReference depthAttachmentRef{};
	depthAttachmentRef.attachment = 1;
	depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	// resolve attachment
	VkAttachmentReference colorAttachmentResolveRef{};
	colorAttachmentResolveRef.attachment = 2;
	colorAttachmentResolveRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	// [subpass 정의]
	VkSubpassDescription subpass{};
	subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpass.colorAttachmentCount = 1; 										// attachment 설정 개수 등록
	subpass.pColorAttachments = &colorAttachmentRef;						// color attachment 등록
	subpass.pDepthStencilAttachment = &depthAttachmentRef;					// depth attachment 등록
	subpass.pResolveAttachments = &colorAttachmentResolveRef;				// resolve attachment 등록

	// [subpass 종속성 설정]
	// 렌더패스 외부 작업(srcSubpass)과 0번 서브패스(dstSubpass) 간의 동기화 설정.
	VkSubpassDependency dependency{};
	dependency.srcSubpass = VK_SUBPASS_EXTERNAL;	// 렌더패스 외부 작업(이전 프레임 처리 또는 렌더패스 외부의 GPU 작업)
	dependency.dstSubpass = 0;					 	// 첫 번째 서브패스(0번 서브패스)에 종속
	// srcStageMask: 동기화를 기다릴 렌더패스 외부 작업의 파이프라인 단계
	dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;	// 색상 첨부물 출력 단계 | 프래그먼트 테스트의 최종 단계
	// srcAccessMask: 렌더패스 외부 작업에서 보장해야 할 메모리 접근 권한
	dependency.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;												// 깊이/스텐실 첨부물 쓰기 권한
	// dstStageMask: 0번 서브패스에서 동기화를 기다릴 파이프라인 단계
	dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;	// 색상 첨부물 출력 단계 | 프래그먼트 테스트의 초기 단계
	// dstAccessMask: 0번 서브패스에서 필요한 메모리 접근 권한
	dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;			// 색상 첨부물 쓰기 권한 | 깊이/스텐실 첨부물 쓰기 권한

	// [렌더 패스 정의]
	std::array<VkAttachmentDescription, 3> attachments = {colorAttachment, depthAttachment, colorAttachmentResolve};
	VkRenderPassCreateInfo renderPassInfo{};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size()); // attachment 설정 개수 등록
	renderPassInfo.pAttachments = attachments.data();							// attachment 설정 등록
	renderPassInfo.subpassCount = 1;											// subpass 개수 등록
	renderPassInfo.pSubpasses = &subpass;										// subpass 등록
	renderPassInfo.dependencyCount = 1;
	renderPassInfo.pDependencies = &dependency;
	
	// [렌더 패스 생성]
	if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS) {
		throw std::runtime_error("failed to create render pass!");
	}
}


/* 
	[디스크립터 셋 레이아웃 생성]
	디스크립터 셋 레이아웃이란? 
	셰이더가 사용할 리소스의 타입과 바인딩 위치를 사전에 정의하는 객체
*/
void Renderer::createDescriptorSetLayout() {
	// 셰이더에 바인딩할 리소스의 종류와 바인딩 위치를 설정할 때 쓰이는 구조체
	VkDescriptorSetLayoutBinding uboLayoutBinding{};
	uboLayoutBinding.binding = 0;														// 바인딩 위치 지정 (디스크립터 셋 내부의 순서)
	uboLayoutBinding.descriptorCount = 1;												// 디스크립터의 개수 (구조체는 1개의 디스크립터 취급, 배열 사용시 여러 개 디스크립터 취급)
	uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;				// 디스크립터의 종류 (현재는 Uniform buffer)
	uboLayoutBinding.pImmutableSamplers = nullptr;										// 변경 불가능한(immutable) 샘플러를 사용할 경우 지정하는 포인터인데 지금은 상관 없음
	uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;							// 사용할 스테이지 지정 (현재는 vertex shader에서 사용하고 여러 스테이지 지정 가능)

	// 디스크립터 셋에 바인딩할 샘플러 설정
	VkDescriptorSetLayoutBinding samplerLayoutBinding{};
	samplerLayoutBinding.binding = 1;													// 바인딩 위치 지정
	samplerLayoutBinding.descriptorCount = 1;											// 디스크립터 개수 지정
	samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;	// 디스크립터의 종류 (현재 Sampler) 
	samplerLayoutBinding.pImmutableSamplers = nullptr;									// 샘플러 불변 설정 (현재 False)
	samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;						// 사용할 스테이지 지정 (Fragment Stage)

	std::array<VkDescriptorSetLayoutBinding, 2> bindings = {uboLayoutBinding, samplerLayoutBinding};	// 바인딩 정보 2개
	// 디스크립터 셋 레이아웃을 생성하기 위한 설정 정보를 포함한 구조체
	VkDescriptorSetLayoutCreateInfo layoutInfo{};
	layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());		// 디스크립터 셋 레이아웃에 포함될 바인딩 정보의 개수
	layoutInfo.pBindings = bindings.data();									// 디스크립터 셋 레이아웃에 포함될 바인딩 정보의 배열

	// 디스크립터 셋 레이아웃 생성
	if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) {
		throw std::runtime_error("failed to create descriptor set layout!");
	}
}

/*
[파이프라인 객체 생성]
파이프라인 객체는 GPU가 그래픽 또는 컴퓨팅 명령을 실행할 때 필요한 설정을 제공한다.
*/ 
void Renderer::createGraphicsPipeline(VkSampleCountFlagBits msaaSamples) {
	// SPIR-V 파일 읽기
	std::vector<char> vertShaderCode = readFile("./shaders/vert.spv");
	std::vector<char> fragShaderCode = readFile("./shaders/frag.spv");

	// shader module 생성
	VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
	VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

	/*
	shader stage 란?
	그래픽 파이프라인에서 사용할 셰이더 단계를 정의하는 구조체
	특정 셰이더 단계에서 사용할 셰이더 코드를 가지고 있음
	*/ 

	// vertex shader stage 설정
	VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
	vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT; // 쉐이더 종류
	vertShaderStageInfo.module = vertShaderModule; // 쉐이더 모듈
	vertShaderStageInfo.pName = "main"; // 쉐이더 파일 내부에서 가장 먼저 시작 될 함수 이름 (엔트리 포인트)

	// fragment shader stage 설정
	VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
	fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT; // 쉐이더 종류
	fragShaderStageInfo.module = fragShaderModule; // 쉐이더 모듈
	fragShaderStageInfo.pName = "main"; // 쉐이더 파일 내부에서 가장 먼저 시작 될 함수 이름 (엔트리 포인트)

	// shader stage 모음
	VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};


	// [vertex 정보 설정]
	VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
	vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

	auto bindingDescription = Vertex::getBindingDescription();												// 정점 바인딩 정보를 가진 구조체
	auto attributeDescriptions = Vertex::getAttributeDescriptions();										// 정점 속성 정보를 가진 구조체 배열

	vertexInputInfo.vertexBindingDescriptionCount = 1;														// 정점 바인딩 정보 개수
	vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());	// 정점 속성 정보 개수
	vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;										// 정점 바인딩 정보
	vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();							// 정점 속성 정보

	// [input assembly 설정] (그려질 primitive 설정)
	VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
	inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST; // primitive로 삼각형 설정
	inputAssembly.primitiveRestartEnable = VK_FALSE; // 인덱스 재시작 x

	/*
	[viewport와 scissor의 설정 값을 정의]
	viewport: 화면좌표로 매핑되어 정규화된 이미지 좌표를, viewport 크기에 맞는 픽셀 좌표로 변경 
				width, height는 (0, 0) ~ (width, height)로 depth는 (0.0f) ~ (1.0f)로 좌표 설정 
	scissor : 픽셀 좌표의 특정 영역에만 렌더링을 하도록 범위를 설정
				픽셀 좌표 내의 offset ~ extent 범위만 렌더링 진행 (나머지는 렌더링 x이므로 쓸데없는 계산 최소화)
	*/ 
	VkPipelineViewportStateCreateInfo viewportState{};
	viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewportState.viewportCount = 1; // 사용할 뷰포트의 수
	viewportState.scissorCount = 1;  // 사용할 시저의수

	// [rasterizer 설정]
	VkPipelineRasterizationStateCreateInfo rasterizer{};
	rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	rasterizer.depthClampEnable = VK_FALSE;  				// VK_FALSE로 설정시 depth clamping이 적용되지 않아 0.0f ~ 1.0f 범위 밖의 프레그먼트는 삭제됨
	rasterizer.rasterizerDiscardEnable = VK_FALSE;  		// rasterization 진행 여부 결정, VK_TRUE시 렌더링 진행 x
	rasterizer.polygonMode = VK_POLYGON_MODE_FILL;  		// 다각형 그리는 방법 선택 (점만, 윤곽선만, 기본 값 등)
	rasterizer.lineWidth = 1.0f;							// 선의 굵기 설정 
	rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;			// cull 모드 설정 (앞면 혹은 뒷면은 그리지 않는 설정 가능)
	rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;	// 앞면의 기준 설정 (y축 반전에 의해 정점이 시계 반대방향으로 그려지므로 앞면을 시계 반대방향으로 설정)
	rasterizer.depthBiasEnable = VK_FALSE;					// depth에 bias를 설정하여 z-fighting 해결할 수 있음 (원근 투영시 멀어질 수록 z값의 차이가 미미해짐)
															// VK_TRUE일 경우 추가 설정 필요

	// [멀티 샘플링 설정]
	VkPipelineMultisampleStateCreateInfo multisampling{};
	multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	multisampling.sampleShadingEnable = VK_TRUE; 		// VK_TRUE: 프레그먼트 셰이더 단계(음영 계산)부터 샘플별로 계산 후 최종 결과 평균내서 사용 
														// VK_FALSE: 테스트&블랜딩 단계부터 샘플별로 계산 후 최종 결과 평균내서 사용 (음영 계산은 동일한 값) 
	multisampling.minSampleShading = 0.2f; 				// 샘플 셰이딩의 최소 비율; 값이 1에 가까울수록 더 부드러워짐
	multisampling.rasterizationSamples = msaaSamples; 	// 픽셀당 샘플 개수 설정

	// [depth test]
	VkPipelineDepthStencilStateCreateInfo depthStencil{};
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;				// 깊이 테스트 활성화 여부를 지정
	depthStencil.depthWriteEnable = VK_TRUE;			// 깊이 버퍼 쓰기 활성화 여부
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;	// 깊이 비교 연산 설정 (VK_COMPARE_OP_LESS: 현재 픽셀의 깊이가 더 작으면 통과)
	depthStencil.depthBoundsTestEnable = VK_FALSE;		// 깊이 범위 테스트 활성화 여부를 지정
	depthStencil.stencilTestEnable = VK_FALSE;			// 스텐실 테스트 활성화 여부를 지정

	// [블랜딩 설정]
	// attachment 별 블랜딩 설정 (블랜딩 + 프레임 버퍼 기록 설정)
	VkPipelineColorBlendAttachmentState colorBlendAttachment{};
	// 프레임 버퍼에 RGBA 값 쓰기 가능 모드 설정 (설정 안 하면 색 수정 x)
	colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachment.blendEnable = VK_FALSE; // 블랜드 기능 off (블랜드 기능 on 시 추가적인 설정 필요)
	// 파이프라인 전체 블랜딩 설정 (attachment 블랜딩 설정 추가)
	VkPipelineColorBlendStateCreateInfo colorBlending{};
	colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	colorBlending.logicOpEnable = VK_FALSE; // 논리 연산 블랜딩 off (블랜딩 대신 논리적 연산을 통해 색을 조합하는 방법으로, 사용시 블렌딩 적용 x)
	colorBlending.logicOp = VK_LOGIC_OP_COPY; // 논리 연산 없이 그냥 전체 복사 (논리 연산 블랜딩이 off면 안 쓰임)
	colorBlending.attachmentCount = 1; // attachment 별 블랜딩 설정 개수
	colorBlending.pAttachments = &colorBlendAttachment; // attachment 별 블랜딩 설정 배열
	// 블랜딩 연산에 사용하는 변수 값 4개 설정 (모든 attachment에 공통으로 사용)
	colorBlending.blendConstants[0] = 0.0f;
	colorBlending.blendConstants[1] = 0.0f;
	colorBlending.blendConstants[2] = 0.0f;
	colorBlending.blendConstants[3] = 0.0f;


	// [파이프라인에서 런타임에 동적으로 상태를 변경할 state 설정]
	std::vector<VkDynamicState> dynamicStates = {
		// Viewport와 Scissor 를 동적 상태로 설정
		VK_DYNAMIC_STATE_VIEWPORT,
		VK_DYNAMIC_STATE_SCISSOR
	};
	VkPipelineDynamicStateCreateInfo dynamicState{};
	dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
	dynamicState.pDynamicStates = dynamicStates.data();


	// [파이프라인 레이아웃 생성]
	VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
	pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutInfo.setLayoutCount = 1; 									// 디스크립터 셋 레이아웃 개수
	pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout; 					// 디스크립투 셋 레이아웃

	if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
		throw std::runtime_error("failed to create pipeline layout!");
	}

	// [파이프라인 정보 생성]
	VkGraphicsPipelineCreateInfo pipelineInfo{};
	pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	pipelineInfo.stageCount = 2; 								// vertex shader, fragment shader 2개 사용
	pipelineInfo.pStages = shaderStages; 						// vertex shader, fragment shader 총 2개의 stageinfo 입력
	pipelineInfo.pVertexInputState = &vertexInputInfo; 			// 정점 정보 입력
	pipelineInfo.pInputAssemblyState = &inputAssembly;			// primitive 정보 입력
	pipelineInfo.pViewportState = &viewportState;				// viewport, scissor 정보 입력
	pipelineInfo.pRasterizationState = &rasterizer;				// 레스터라이저 설정 입력
	pipelineInfo.pMultisampleState = &multisampling;			// multisampling 설정 입력
	pipelineInfo.pDepthStencilState = &depthStencil;			// depth-stencil 설정
	pipelineInfo.pColorBlendState = &colorBlending;				// 블랜딩 설정 입력
	pipelineInfo.pDynamicState = &dynamicState;					// 동적으로 변경할 상태 입력
	pipelineInfo.layout = pipelineLayout;						// 파이프라인 레이아웃 설정 입력
	pipelineInfo.renderPass = renderPass;						// 렌더패스 입력
	pipelineInfo.subpass = 0;									// 렌더패스 내 서브패스의 인덱스
	pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;			// 상속을 위한 기존 파이프라인 핸들
	pipelineInfo.basePipelineIndex = -1; 						// Optional (상속을 위한 기존 파이프라인 인덱스)	

	// [파이프라인 객체 생성]
	// 두 번째 매개변수는 상속할 파이프라인
	if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS) {
		throw std::runtime_error("failed to create graphics pipeline!");
	}

	// 쉐이더 모듈 더 안 쓰므로 제거
	vkDestroyShaderModule(device, fragShaderModule, nullptr);
	vkDestroyShaderModule(device, vertShaderModule, nullptr);
}

/*
매개변수로 받은 쉐이더 파일을 shader module로 만들어 줌
shader module은 쉐이더 파일을 객체화 한 것임
*/ 
VkShaderModule Renderer::createShaderModule(const std::vector<char>& code) {
	VkShaderModuleCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.codeSize = code.size();									// 코드 길이 입력
	createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());	// 코드 내용 입력

	// 쉐이더 모듈 생성
	VkShaderModule shaderModule;
	if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
		throw std::runtime_error("failed to create shader module!");
	}

	return shaderModule;
}

VkRenderPass Renderer::getRenderPass()
{
	return renderPass;
}

VkDescriptorSetLayout Renderer::getDescriptorSetLayout()
{
	return descriptorSetLayout;
}

VkPipelineLayout Renderer::getPipelineLayout()
{
	return pipelineLayout;
}

VkPipeline Renderer::getGraphicsPipeline()
{
	return graphicsPipeline;
}
