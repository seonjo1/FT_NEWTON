#include "../include/model.h"

std::unique_ptr<Model> Model::create(std::string filename, DeviceManager *deviceManager, VkCommandPool commandPool)
{
	std::unique_ptr<Model> model(new Model());
	model->load(filename, deviceManager, commandPool);
	return model;
}

std::unique_ptr<Model> Model::createBox(DeviceManager *deviceManager, VkCommandPool commandPool,
										const ale::Transform &xf, std::string diffusePath, std::string specularPath)
{
	std::unique_ptr<Model> box(new Model());
	box->createBoxMesh(deviceManager, commandPool, diffusePath, specularPath, xf);

	return box;
}

void Model::createBoxMesh(DeviceManager *deviceManager, VkCommandPool commandPool, std::string diffusePath,
						  std::string specularPath, const ale::Transform &xf)
{
	shape = new ale::BoxShape();
	ale::BoxShape *boxShape = dynamic_cast<ale::BoxShape *>(shape);

	if (boxShape)
	{
		size = 1;
		materials.push_back(Material::create(deviceManager, commandPool, diffusePath, specularPath));
		meshes.push_back(Mesh::createBox(deviceManager, commandPool, boxShape, xf));
		meshes[0]->setMaterial(materials[0].get());
		m_isStatic = false;
	}
}

std::unique_ptr<Model> Model::createSphere(DeviceManager *deviceManager, VkCommandPool commandPool,
										   const ale::Transform &xf, std::string diffusePath, std::string specularPath)
{
	std::unique_ptr<Model> sphere(new Model());
	sphere->createSphereMesh(deviceManager, commandPool, diffusePath, specularPath, xf);
	return sphere;
}

std::unique_ptr<Model> Model::createGround(DeviceManager *deviceManager, VkCommandPool commandPool,
										   const ale::Transform &xf, std::string diffusePath, std::string specularPath)
{
	std::unique_ptr<Model> ground(new Model());
	ground->createGroundMesh(deviceManager, commandPool, diffusePath, specularPath, xf);
	return ground;
}

void Model::createSphereMesh(DeviceManager *deviceManager, VkCommandPool commandPool, std::string diffusePath,
							 std::string specularPath, const ale::Transform &xf)
{
	shape = new ale::SphereShape();
	ale::SphereShape *sphereShape = dynamic_cast<ale::SphereShape *>(shape);

	if (sphereShape)
	{
		size = 1;
		materials.push_back(Material::create(deviceManager, commandPool, diffusePath, specularPath));
		meshes.push_back(Mesh::createSphere(deviceManager, commandPool, sphereShape, xf));
		meshes[0]->setMaterial(materials[0].get());
		m_isStatic = false;
	}
}

void Model::createGroundMesh(DeviceManager *deviceManager, VkCommandPool commandPool, std::string diffusePath,
							 std::string specularPath, const ale::Transform &xf)
{
	shape = new ale::BoxShape();
	ale::BoxShape *groundShape = dynamic_cast<ale::BoxShape *>(shape);

	shape->setType(ale::Type::GROUND);
	if (groundShape)
	{
		size = 1;
		materials.push_back(Material::create(deviceManager, commandPool, diffusePath, specularPath));
		meshes.push_back(Mesh::createGround(deviceManager, commandPool, groundShape, xf));
		meshes[0]->setMaterial(materials[0].get());
		m_isStatic = true;
	}
}

void Model::load(std::string filename, DeviceManager *deviceManager, VkCommandPool commandPool)
{
	Assimp::Importer importer;
	// scene 구조체 받아오기
	const aiScene *scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_FlipUVs);
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
		aiMaterial *materialInfo = scene->mMaterials[i];
		materials.push_back(Material::create(materialInfo, deviceManager, commandPool, dirname));
	}

	// node 데이터 처리
	processNode(deviceManager, commandPool, scene->mRootNode, scene);

	size = meshes.size();
}

void Model::processNode(DeviceManager *deviceManager, VkCommandPool commandPool, aiNode *node, const aiScene *scene)
{
	// node에 포함된 mesh들 순회
	for (uint32_t i = 0; i < node->mNumMeshes; i++)
	{
		// 현재 처리할 mesh 찾기
		uint32_t meshIndex = node->mMeshes[i];
		aiMesh *mesh = scene->mMeshes[meshIndex];
		// 현재 mesh 데이터 처리
		processMesh(deviceManager, commandPool, mesh, scene);
	}

	// 자식 노드 처리
	for (uint32_t i = 0; i < node->mNumChildren; i++)
		processNode(deviceManager, commandPool, node->mChildren[i], scene);
}

void Model::processMesh(DeviceManager *deviceManager, VkCommandPool commandPool, aiMesh *mesh, const aiScene *scene)
{
	std::vector<Vertex> vertices;
	vertices.resize(mesh->mNumVertices);
	for (uint32_t i = 0; i < mesh->mNumVertices; i++)
	{
		Vertex &v = vertices[i];
		v.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
		v.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
		v.texCoord = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
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

	std::unique_ptr<Mesh> newMesh = Mesh::create(deviceManager, commandPool, vertices, indices);
	// mesh의 mMaterialINdex가 0이상이면 이 mesh는 material을 갖고 있으므로
	// 해당 material값을 setting 해준다.
	if (mesh->mMaterialIndex >= 0)
		newMesh->setMaterial(materials[mesh->mMaterialIndex].get());
	meshes.push_back(std::move(newMesh));
}

void Model::clear()
{
	for (std::unique_ptr<Mesh> &mesh : meshes)
	{
		mesh->clear();
	}

	for (std::unique_ptr<Material> &material : materials)
	{
		material->clear();
	}
}

void Model::recordDrawCommand(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, uint32_t currentFrame)
{
	for (std::unique_ptr<Mesh> &mesh : meshes)
	{
		mesh->recordDrawCommand(commandBuffer, pipelineLayout, currentFrame);
	}
}

void Model::updateUniformBuffer(UniformBufferObject &ubo, uint32_t currentFrame)
{
	for (std::unique_ptr<Mesh> &mesh : meshes)
	{
		mesh->getUniformBuffer()->update(ubo, currentFrame);
	}
}

void Model::createDescriptorSets(VkDevice device, VkDescriptorPool descriptorPool,
								 VkDescriptorSetLayout descriptorSetLayout)
{
	for (std::unique_ptr<Mesh> &mesh : meshes)
	{
		mesh->createDescriptorSets(device, descriptorPool, descriptorSetLayout);
	}
}

uint32_t Model::getSize()
{
	return size;
}

ale::Shape *Model::getShape() const
{
	return shape;
}

bool Model::isStatic()
{
	return m_isStatic;
}

