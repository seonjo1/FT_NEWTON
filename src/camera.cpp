#include "../include/camera.h"

void Camera::rotate(glm::vec2& pos) {
	glm::vec2 deltaPos = pos - prevMousePos;

	cameraYaw -= deltaPos.x * cameraRotSpeed;
	cameraPitch -= deltaPos.y * cameraRotSpeed;

	if (cameraYaw < 0.0f) { cameraYaw += 360.0f; }
	if (cameraYaw > 360.0f) { cameraYaw -= 360.0f; }

	if (cameraPitch > 89.0f) { cameraPitch = 89.0f; }
	if (cameraPitch < -89.0f) { cameraPitch = -89.0f; }

	prevMousePos = pos;
}

void Camera::move(bool pressW, bool pressS, bool pressD,
					bool pressA, bool pressE, bool pressQ) {

	glm::vec3 frontVector = cameraFront;
	glm::vec3 rightVector = glm::normalize(glm::cross(cameraUp, -frontVector));
	glm::vec3 upVector = glm::normalize(glm::cross(-frontVector, rightVector));
	
	if (pressW) {
		cameraPos = cameraPos + cameraSpeed * frontVector;
	}

	if (pressS) {
		cameraPos = cameraPos - cameraSpeed * frontVector;
	}

	if (pressD) {
		cameraPos = cameraPos + cameraSpeed * rightVector;
	}

	if (pressA) {
		cameraPos = cameraPos - cameraSpeed * rightVector;
	}

	if (pressE) {
		cameraPos = cameraPos + cameraSpeed * upVector;
	}

	if (pressQ) {
		cameraPos = cameraPos - cameraSpeed * upVector;
	}
}

void Camera::saveCurrentPos(float x, float y) {
	prevMousePos = glm::vec2((float)x, (float)y);
}

glm::mat4 Camera::getViewMatrix() {
    // Pitch와 Yaw를 라디안 단위로 변환
 	glm::vec3 eulerAngleRadians(glm::radians(cameraPitch), glm::radians(cameraYaw), 0.0f);

    // Euler 각도 -> 쿼터니언 변환
    glm::quat rotationQuat = glm::quat(eulerAngleRadians);

    // 쿼터니언을 행렬로 변환
    glm::mat4 rotationMatrix = glm::mat4_cast(rotationQuat);

    // 기본 방향 벡터
    glm::vec4 defaultFront(0.0f, 0.0f, -1.0f, 0.0f);

    // cameraFront 계산
    glm::vec4 rotatedFront = rotationMatrix * defaultFront;
    cameraFront = glm::normalize(glm::vec3(rotatedFront));
	
	return glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
}

glm::vec3 Camera::getCameraPos()
{
	return cameraPos;
}

glm::vec3 Camera::getCameraFront()
{
	return cameraFront;
}
