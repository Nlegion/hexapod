#include "SafetyMonitor.h"
#include "config.h"

SafetyMonitor::SafetyMonitor() {
    for(int i = 0; i < 32; i++) {
        limits[i] = {500, 2500, 500};
        positions[i] = DEFAULT_SERVO_POS;
    }
}

bool SafetyMonitor::validate_leg_position(int leg, int StepSpeed) {
    int base = leg * 3;
    return validate_command(base, positions[base], StepSpeed) &&
           validate_command(base+1, positions[base+1], StepSpeed) &&
           validate_command(base+2, positions[base+2], StepSpeed);
}
