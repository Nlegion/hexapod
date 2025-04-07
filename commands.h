#pragma once
#include "config.h"
#include "safety.h"  // Добавляем include ПЕРЕД использованием SafetySystem
#include "logger.h"

class Commands {
public:
    static void send_servo(int servo, int pulse) {
        pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);

        // Удалите условие, блокирующее граничные значения
        if (servo < 0 || servo >= TOTAL_LEGS * SERVOS_PER_LEG) return;

        char buffer[32];
        snprintf(buffer, sizeof(buffer), "#%dP%dT%d\r\n", 
                servo, pulse, SafetySystem::get_speed());
        Serial1.print(buffer);
    }
};
