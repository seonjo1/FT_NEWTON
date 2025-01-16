cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME VULKAN_TUTORIAL)
set(CMAKE_CXX_STANDARD 17)
 
project(${PROJECT_NAME})
set(SRC src/main.cpp src/App.cpp src/VulkanInstance.cpp src/DeviceManager.cpp
		src/SwapChainManager.cpp src/Image.cpp src/Renderer.cpp
		src/Buffer.cpp src/Mesh.cpp src/Model.cpp src/SyncObject.cpp 
		src/CommandManager.cpp src/DescriptorPool.cpp src/Camera.cpp
		src/physics/BoxShape.cpp src/physics/BoxToBoxContact.cpp src/physics/BroadPhase.cpp 
		src/physics/Contact.cpp src/physics/ContactManager.cpp src/physics/ContactSolver.cpp 
		src/physics/DynamicTree.cpp src/physics/Fixture.cpp src/physics/Island.cpp 
		src/physics/Rigidbody.cpp src/physics/Shape.cpp src/physics/SphereShape.cpp 
		src/physics/SphereToBoxContact.cpp src/physics/SphereToSphereContact.cpp src/physics/World.cpp 
		src/physics/SphereToCylinderContact.cpp src/physics/CylinderToCylinderContact.cpp
		src/physics/BoxToCylinderContact.cpp src/physics/CylinderShape.cpp
		src/physics/CapsuleToCapsuleContact.cpp src/physics/CapsuleShape.cpp
		src/physics/SphereToCapsuleContact.cpp src/physics/CylinderToCapsuleContact.cpp 
		src/physics/BoxToCapsuleContact.cpp	)

add_executable(${PROJECT_NAME} ${SRC})

include(Dependency.cmake)

# 우리 프로젝트에 include / lib 관련 옵션 추가

set(CMAKE_PREFIX_PATH "C:/VulkanSDK/1.3.296.0")
find_package(Vulkan REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Vulkan::Vulkan)

set(INCLUDE_DIR include)
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${DEP_INCLUDE_DIR})
target_link_directories(${PROJECT_NAME} PUBLIC ${DEP_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_LIBS})

target_compile_options(${PROJECT_NAME} PUBLIC "/utf-8")

# Dependency들이 먼저 build 될 수 있게 관계 설정 / 뒤에서 부터 컴파일
add_dependencies(${PROJECT_NAME} ${DEP_LIST})
