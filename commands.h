#pragma once
#include "config.h"
#include "safety.h"  // Добавляем include ПЕРЕД использованием SafetySystem
#include "logger.h"

class Commands {
public:
    static void send_servo(int servo, int pulse) {
        if(servo < 0 || servo >= NUM_LEGS*SERVOS_PER_LEG) return;
        
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "#%dP%dT%d\r\n", 
                servo+1, pulse, SafetySystem::get_speed());
        
        Serial1.print(buffer);
        Logger::log(Logger::INFO, "Sent: %s", buffer);
    }
};
