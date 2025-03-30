#ifndef SERVOMANAGER_H
#define SERVOMANAGER_H

#include <Arduino.h>
#include "config.h"

class ServoManager {
private:
    int positions[32] = {DEFAULT_SERVO_POS};
    int currentPositions[32] = {0};

public:
    void init();
    void set_position(int channel, int value);
    String generate_command(int speed);
};

extern ServoManager servoManager;

#endif
