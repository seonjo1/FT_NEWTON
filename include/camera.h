#ifndef CAMERA_H
# define CAMERA_H

# include "common.h"

class Camera {
public:
	Camera() = default;
	void move(bool pressW, bool pressS, bool pressD,
			  bool pressA, bool pressE, bool pressQ);
	void rotate(glm::vec2& pos);
	void saveCurrentPos(float x, float y);
	glm::mat4 getViewMatrix();
	glm::vec3 getCameraPos();
	glm::vec3 getCameraFront();

private:
	float cameraPitch { 0.0f };
	float cameraYaw { 0.0f };
	glm::vec2 prevMousePos { glm::vec2(0.0f)};
	glm::vec3 cameraPos {0.0f, 0.0f, 10.0f};
	glm::vec3 cameraFront {0.0f, 0.0f, -1.0f};
	glm::vec3 cameraUp {0.0f, 1.0f, 0.0f};
	const float cameraSpeed { 0.01f };
	const float cameraRotSpeed { 0.1f };
};

#endif