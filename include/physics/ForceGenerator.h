#ifndef FORCEGENERATOR_H
#define FORCEGENERATOR_H

class Rigidbody;

class ForceGenerator
{
public:
    virtual void updateForce(Rigidbody* body, float duration) = 0;
};

#endif