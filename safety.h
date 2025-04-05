#pragma once
#include "config.h"
#include "logger.h"

class SafetySystem {
public:
    static int get_speed();
    static void init();
    static bool set_servo(int servo, int pulse);
    static void update_load_monitor();
    static void emergency_stop();
    static void set_speed(float speed);

private:
    static float read_current(int leg);
    static float current_load[NUM_LEGS];
    static float max_speed;
    static unsigned long last_update;
};
