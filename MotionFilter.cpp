#include "MotionFilter.h"

MotionFilter::MotionFilter(float time_constant) 
    : tau(time_constant), 
      filtered{0,0,0}, 
      last_update(micros()) {}

Vector3D MotionFilter::update(const Vector3D& target) {
    uint64_t now = micros();
    float dt = (now - last_update) * 1e-6f;
    last_update = now;
    
    filtered.x += (target.x - filtered.x) * (dt / (tau + dt));
    filtered.y += (target.y - filtered.y) * (dt / (tau + dt));
    filtered.z += (target.z - filtered.z) * (dt / (tau + dt));
    return filtered;
}
