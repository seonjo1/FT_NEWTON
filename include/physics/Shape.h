#ifndef SHAPE_H
#define SHAPE_H

#include <glm/glm.hpp>

namespace ale
{
struct MassData
{
    glm::vec3 center;
    float mass;
    // inertia
    float I;
};

class Shape
{
public:
    enum class Type
    {
        e_sphere = 0,
        e_box = 1
    };

    virtual ~Shape();
    virtual Shape* clone() const = 0;
    virtual int GetChildCount() const = 0;
    virtual void ComputeAABB() const = 0;
    virtual void ComputeMass() const= 0;
};
}

#endif