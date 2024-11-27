#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class Fixture;

class Rigidbody
{
public:
    Rigidbody();
    void integrate();
    void calculateDerivedData();
    void addForce();
    void addForceAtPoint();
    void addForceAtBodyPoint();
    void addTorque();
    void clearAccumulators();

protected:
    glm::vec3 position;
    glm::quat orientation;
    glm::vec3 rotation;
    glm::mat3 inverseInertiaTensorWorld;
    float motion;
    bool isAwake;
    bool canSleep;
    glm::mat4 transformMatrix;
    glm::vec3 forceAccum;
    glm::vec3 torqueAccum;
    glm::vec3 acceleration;
    glm::vec3 lastFrameAcceleration;
    // std::vector<Fixture*> fixtures;

private:

};

#endif