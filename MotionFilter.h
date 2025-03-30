#pragma once
#include <Arduino.h>

struct Vector3D {
    float x;
    float y;
    float z;
};

class MotionFilter {
public:
    MotionFilter(float time_constant = 0.1f);
    Vector3D update(const Vector3D& target);
    
private:
    float tau;
    Vector3D filtered;
    uint64_t last_update;
};
