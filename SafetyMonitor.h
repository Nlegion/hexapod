#pragma once
#include <Arduino.h>
#include "config.h"

class SafetyMonitor {
private:
    struct ServoLimits {
        int min;
        int max;
        int maxSpeed;
    };

    ServoLimits limits[32];
    int positions[32]; // Добавляем массив позиций

public:
    SafetyMonitor();
    bool validate_command(int servo, int target, int speed);
    bool validate_leg_position(int leg, int StepSpeed); // Добавляем параметр скорости
};
