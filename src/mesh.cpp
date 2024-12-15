#include "../include/mesh.h"

std::unique_ptr<Mesh> Mesh::create(DeviceManager *deviceManager, VkCommandPool commandPool,
								   const std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices)
{
	std::unique_ptr<Mesh> mesh(new Mesh());
	mesh->init(deviceManager, commandPool, vertices, indices);
	return mesh;
}

void Mesh::init(DeviceManager *deviceManager, VkCommandPool commandPool, const std::vector<Vertex> &vertices,
				const std::vector<uint32_t> &indices)
{
	vertexBuffer = VertexBuffer::create(vertices, deviceManager, commandPool);
	indexBuffer = IndexBuffer::create(indices, deviceManager, commandPool);
	uniformBuffer = UniformBuffer::create(deviceManager, sizeof(UniformBufferObject));
}

void Mesh::setMaterial(Material *material)
{
	this->material = material;
}

// 디스크립터 셋 할당 및 업데이트 하여 리소스 바인딩
void Mesh::createDescriptorSets(VkDevice device, VkDescriptorPool descriptorPool,
								VkDescriptorSetLayout descriptorSetLayout)
{
	// 디스크립터 셋 레이아웃 벡터 생성 (기존 만들어놨던 디스크립터 셋 레이아웃 객체 이용)
	std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, descriptorSetLayout);

	// 디스크립터 셋 할당에 필요한 정보를 설정하는 구조체
	VkDescriptorSetAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = descriptorPool; // 디스크립터 셋을 할당할 디스크립터 풀 지정
	allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT); // 할당할 디스크립터 셋 개수 지정
	allocInfo.pSetLayouts = layouts.data(); // 할당할 디스크립터 셋 의 레이아웃을 정의하는 배열

	descriptorSets.resize(MAX_FRAMES_IN_FLIGHT); // 디스크립터 셋을 저장할 벡터 크기 설정

	// 디스크립터 풀에 디스크립터 셋 할당
	if (vkAllocateDescriptorSets(device, &allocInfo, descriptorSets.data()) != VK_SUCCESS)
	{
		throw std::runtime_error("failed to allocate descriptor sets!");
	}

	std::vector<VkBuffer> &uniformBuffers = uniformBuffer->getBuffers();

	// 디스크립터 셋마다 디스크립터 설정 진행
	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
	{
		// 디스크립터 셋에 바인딩할 버퍼 정보
		VkDescriptorBufferInfo bufferInfo{};
		bufferInfo.buffer = uniformBuffers[i];			// 바인딩할 버퍼
		bufferInfo.offset = 0;							// 버퍼에서 데이터 시작 위치 offset
		bufferInfo.range = sizeof(UniformBufferObject); // 셰이더가 접근할 버퍼 크기

		VkDescriptorImageInfo diffuseImageInfo{};
		diffuseImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; // 이미지의 레이아웃
		diffuseImageInfo.imageView = material->diffuse->getImageView(); // 셰이더에서 사용할 이미지 뷰
		diffuseImageInfo.sampler = material->diffuse->getSampler(); // 이미지 샘플링에 사용할 샘플러 설정

		VkDescriptorImageInfo specularImageInfo{};
		specularImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; // 이미지의 레이아웃
		specularImageInfo.imageView = material->specular->getImageView(); // 셰이더에서 사용할 이미지 뷰
		specularImageInfo.sampler = material->specular->getSampler(); // 이미지 샘플링에 사용할 샘플러 설정

		// 디스크립터 셋 바인딩 및 업데이트
		std::array<VkWriteDescriptorSet, 3> descriptorWrites{};

		descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[0].dstSet = descriptorSets[i]; // 업데이트 할 디스크립터 셋
		descriptorWrites[0].dstBinding = 0;				// 업데이트 할 바인딩 포인트
		descriptorWrites[0].dstArrayElement = 0; // 업데이트 할 디스크립터가 배열 타입인 경우 해당 배열의 원하는 index
												 // 부터 업데이트 가능 (배열 아니면 0으로 지정)
		descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER; // 업데이트 할 디스크립터 타입
		descriptorWrites[0].descriptorCount = 1;	   // 업데이트 할 디스크립터 개수
		descriptorWrites[0].pBufferInfo = &bufferInfo; // 업데이트 할 버퍼 디스크립터 정보 구조체 배열

		descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[1].dstSet = descriptorSets[i]; // 업데이트 할 디스크립터 셋
		descriptorWrites[1].dstBinding = 1;				// 업데이트 할 바인딩 포인트
		descriptorWrites[1].dstArrayElement = 0; // 업데이트 할 디스크립터가 배열 타입인 경우 해당 배열의 원하는 index
												 // 부터 업데이트 가능 (배열 아니면 0으로 지정)
		descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; // 업데이트 할 디스크립터 타입
		descriptorWrites[1].descriptorCount = 1;			// 업데이트 할 디스크립터 개수
		descriptorWrites[1].pImageInfo = &diffuseImageInfo; // 업데이트 할 버퍼 디스크립터 정보 구조체 배열

		descriptorWrites[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[2].dstSet = descriptorSets[i]; // 업데이트 할 디스크립터 셋
		descriptorWrites[2].dstBinding = 2;				// 업데이트 할 바인딩 포인트
		descriptorWrites[2].dstArrayElement = 0; // 업데이트 할 디스크립터가 배열 타입인 경우 해당 배열의 원하는 index
												 // 부터 업데이트 가능 (배열 아니면 0으로 지정)
		descriptorWrites[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; // 업데이트 할 디스크립터 타입
		descriptorWrites[2].descriptorCount = 1; // 업데이트 할 디스크립터 개수
		descriptorWrites[2].pImageInfo = &specularImageInfo;

		// 디스크립터 셋을 업데이트 하여 사용할 리소스 바인딩
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0,
							   nullptr);
	}
}

VertexBuffer *Mesh::getVertexBuffer()
{
	return vertexBuffer.get();
}

IndexBuffer *Mesh::getIndexBuffer()
{
	return indexBuffer.get();
}

UniformBuffer *Mesh::getUniformBuffer()
{
	return uniformBuffer.get();
}

void Mesh::clear()
{
	vertexBuffer->clear();
	indexBuffer->clear();
	uniformBuffer->clear();
}

void Mesh::recordDrawCommand(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, uint32_t currentFrame)
{
	// 버텍스 정보 입력
	VkDeviceSize offsets[] = {0}; // 버텍스 버퍼 메모리의 시작 위치 offset
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffer->getBuffers().data(),
						   offsets); // 커맨드 버퍼에 버텍스 버퍼 바인딩

	// 인덱스 정보 입력
	vkCmdBindIndexBuffer(
		commandBuffer, indexBuffer->getBuffer(), 0,
		VK_INDEX_TYPE_UINT32); // 커맨드 버퍼에 인덱스 버퍼 바인딩 (4번째 매개변수 index 데이터 타입 uint32 설정)

	// 디스크립터 셋을 커맨드 버퍼에 바인딩
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1,
							&descriptorSets[currentFrame], 0, nullptr);

	// [Drawing 작업을 요청하는 명령 기록]
	vkCmdDrawIndexed(commandBuffer, indexBuffer->getIndicesSize(), 1, 0, 0, 0); // index로 drawing 하는 명령 기록
}

std::unique_ptr<Material> Material::create(aiMaterial *materialInfo, DeviceManager *deviceManager,
										   VkCommandPool commandPool, std::string &dirname)
{
	std::unique_ptr<Material> material(new Material());
	material->init(materialInfo, deviceManager, commandPool, dirname);
	return material;
}

std::unique_ptr<Material> Material::create(DeviceManager *deviceManager, VkCommandPool commandPool,
										   std::string diffusePath, std::string specularPath)
{
	std::unique_ptr<Material> material(new Material());
	material->init(deviceManager, commandPool, diffusePath, specularPath);
	return material;
}

void Material::init(aiMaterial *materialInfo, DeviceManager *deviceManager, VkCommandPool commandPool,
					std::string &dirname)
{
	std::optional<std::string> diffusePath = getFilePath(materialInfo, aiTextureType_DIFFUSE, dirname);
	std::optional<std::string> specularPath = getFilePath(materialInfo, aiTextureType_SPECULAR, dirname);

	if (diffusePath.has_value())
		diffuse = TextureImage::create(diffusePath.value().c_str(), deviceManager, commandPool);
	else
		diffuse = TextureImage::createBlackTexture(deviceManager, commandPool);

	if (specularPath.has_value())
		specular = TextureImage::create(specularPath.value().c_str(), deviceManager, commandPool);
	else
		specular = TextureImage::createBlackTexture(deviceManager, commandPool);
}

void Material::init(DeviceManager *deviceManager, VkCommandPool commandPool, std::string diffusePath,
					std::string specularPath)
{
	if (diffusePath == "black")
		diffuse = TextureImage::createBlackTexture(deviceManager, commandPool);
	else
		diffuse = TextureImage::create(diffusePath.c_str(), deviceManager, commandPool);

	if (specularPath == "black")
		specular = TextureImage::createBlackTexture(deviceManager, commandPool);
	else
		specular = TextureImage::create(specularPath.c_str(), deviceManager, commandPool);
}

std::optional<std::string> Material::getFilePath(aiMaterial *materialInfo, aiTextureType type, std::string &dirname)
{
	if (materialInfo->GetTextureCount(type) <= 0)
		return {};
	// 해당 type의 image 파일 이름을 filepath에 받아옴
	aiString filepath;
	materialInfo->GetTexture(type, 0, &filepath);
	// filepath를 directory 경로와 합체해서 load
	std::filesystem::path fullPath = std::filesystem::path(dirname) / filepath.C_Str();
	return fullPath.string();
}

void Material::clear()
{
	diffuse->clear();
	specular->clear();
}

std::unique_ptr<Mesh> Mesh::createBox(DeviceManager *deviceManager, VkCommandPool commandPool, ale::BoxShape *shape,
									  const ale::Transform &xf)
{
	// 인자로 받은 transform에 따라 vertex 다르게 생성해야 함
	std::vector<Vertex> vertices = {
		Vertex{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(0.0f, 0.0f)},
		Vertex{glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(1.0f, 0.0f)},
		Vertex{glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(1.0f, 1.0f)},
		Vertex{glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(0.0f, 1.0f)},

		Vertex{glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)},
		Vertex{glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 0.0f)},
		Vertex{glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 1.0f)},
		Vertex{glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 1.0f)},

		Vertex{glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
		Vertex{glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
		Vertex{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 1.0f)},
		Vertex{glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f)},

		Vertex{glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
		Vertex{glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
		Vertex{glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 1.0f)},
		Vertex{glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f)},

		Vertex{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(0.0f, 1.0f)},
		Vertex{glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
		Vertex{glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
		Vertex{glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(0.0f, 0.0f)},

		Vertex{glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(0.0f, 1.0f)},
		Vertex{glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
		Vertex{glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
		Vertex{glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(0.0f, 0.0f)},
	};

	shape->center = xf.position;
	shape->SetVertices(vertices);

	std::vector<uint32_t> indices = {
		0,	2,	1,	2,	0,	3,	4,	5,	6,	6,	7,	4,	8,	9,	10, 10, 11, 8,
		12, 14, 13, 14, 12, 15, 16, 17, 18, 18, 19, 16, 20, 22, 21, 22, 20, 23,
	};

	return create(deviceManager, commandPool, vertices, indices);
}

std::unique_ptr<Mesh> Mesh::createSphere(DeviceManager *deviceManager, VkCommandPool commandPool,
										 ale::SphereShape *shape, const ale::Transform &xf)
{
	std::vector<Vertex> vertices;
	std::vector<uint32_t> indices;
	uint32_t latiSegmentCount = 16;
	uint32_t longiSegmentCount = 32;

	uint32_t circleVertCount = longiSegmentCount + 1;
	vertices.resize((latiSegmentCount + 1) * circleVertCount);
	// vertex 위치 수정 필요
	for (uint32_t i = 0; i <= latiSegmentCount; i++)
	{
		float v = (float)i / (float)latiSegmentCount;
		float phi = (v - 0.5f) * glm::pi<float>();
		auto cosPhi = cosf(phi);
		auto sinPhi = sinf(phi);
		for (uint32_t j = 0; j <= longiSegmentCount; j++)
		{
			float u = (float)j / (float)longiSegmentCount;
			float theta = u * glm::pi<float>() * 2.0f;
			auto cosTheta = cosf(theta);
			auto sinTheta = sinf(theta);
			auto point = glm::vec3(cosPhi * cosTheta, sinPhi, -cosPhi * sinTheta);

			vertices[i * circleVertCount + j] = Vertex{point * 0.5f, point, glm::vec2(u, v)};
		}
	}

	shape->center = xf.position;
	shape->SetRadius(0.5f);
	shape->setShapeFeatures(vertices);

	indices.resize(latiSegmentCount * longiSegmentCount * 6);
	for (uint32_t i = 0; i < latiSegmentCount; i++)
	{
		for (uint32_t j = 0; j < longiSegmentCount; j++)
		{
			uint32_t vertexOffset = i * circleVertCount + j;
			uint32_t indexOffset = (i * longiSegmentCount + j) * 6;
			indices[indexOffset] = vertexOffset;
			indices[indexOffset + 1] = vertexOffset + 1;
			indices[indexOffset + 2] = vertexOffset + 1 + circleVertCount;
			indices[indexOffset + 3] = vertexOffset;
			indices[indexOffset + 4] = vertexOffset + 1 + circleVertCount;
			indices[indexOffset + 5] = vertexOffset + circleVertCount;
		}
	}

	return create(deviceManager, commandPool, vertices, indices);
}

std::unique_ptr<Mesh> Mesh::createGround(DeviceManager *deviceManager, VkCommandPool commandPool, ale::BoxShape *shape,
										 const ale::Transform &xf)
{
	// 인자로 받은 transform에 따라 vertex 다르게 생성해야 함
	std::vector<Vertex> vertices = {
		Vertex{glm::vec3(-100.0f, -0.01f, -100.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(0.0f, 0.0f)},
		Vertex{glm::vec3(100.0f, -0.01f, -100.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(100.0f, 0.0f)},
		Vertex{glm::vec3(100.0f, 0.01f, -100.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(100.0f, 100.0f)},
		Vertex{glm::vec3(-100.0f, 0.01f, -100.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(0.0f, 100.0f)},

		Vertex{glm::vec3(-100.0f, -0.01f, 100.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)},
		Vertex{glm::vec3(100.0f, -0.01f, 100.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(100.0f, 0.0f)},
		Vertex{glm::vec3(100.0f, 0.01f, 100.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(100.0f, 100.0f)},
		Vertex{glm::vec3(-100.0f, 0.01f, 100.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 100.0f)},

		Vertex{glm::vec3(-100.0f, 0.01f, 100.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(100.0f, 0.0f)},
		Vertex{glm::vec3(-100.0f, 0.01f, -100.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(100.0f, 100.0f)},
		Vertex{glm::vec3(-100.0f, -0.01f, -100.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 100.0f)},
		Vertex{glm::vec3(-100.0f, -0.01f, 100.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f)},

		Vertex{glm::vec3(100.0f, 0.01f, 100.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(100.0f, 0.0f)},
		Vertex{glm::vec3(100.0f, 0.01f, -100.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(100.0f, 100.0f)},
		Vertex{glm::vec3(100.0f, -0.01f, -100.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 100.0f)},
		Vertex{glm::vec3(100.0f, -0.01f, 100.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f)},

		Vertex{glm::vec3(-100.0f, -0.01f, -100.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(0.0f, 100.0f)},
		Vertex{glm::vec3(100.0f, -0.01f, -100.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(100.0f,100.0f)},
		Vertex{glm::vec3(100.0f, -0.01f, 100.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(100.0f, 0.0f)},
		Vertex{glm::vec3(-100.0f, -0.01f, 100.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(0.0f, 0.0f)},

		Vertex{glm::vec3(-100.0f, 0.01f, -100.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(0.0f, 100.0f)},
		Vertex{glm::vec3(100.0f, 0.01f, -100.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(100.0f, 100.0f)},
		Vertex{glm::vec3(100.0f, 0.01f, 100.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(100.0f, 0.0f)},
		Vertex{glm::vec3(-100.0f, 0.01f, 100.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(0.0f, 0.0f)},
	};

	shape->center = xf.position;
	shape->SetVertices(vertices);

	std::vector<uint32_t> indices = {
		0,	2,	1,	2,	0,	3,	4,	5,	6,	6,	7,	4,	8,	9,	10, 10, 11, 8,
		12, 14, 13, 14, 12, 15, 16, 17, 18, 18, 19, 16, 20, 22, 21, 22, 20, 23,
	};

	return create(deviceManager, commandPool, vertices, indices);
}