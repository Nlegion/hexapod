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

#include "commands.h"

inline int SafetySystem::get_speed() {
    return static_cast<int>(max_speed * 1000.0f / MAX_SPEED);
}

float SafetySystem::current_load[NUM_LEGS];
float SafetySystem::max_speed = MAX_SPEED;
unsigned long SafetySystem::last_update = 0;

void SafetySystem::init() {
    for (int i = 0; i < NUM_LEGS; i++)
        current_load[i] = 0.0f;
    max_speed = MAX_SPEED;
    pinMode(A0, ANALOG);
}

bool SafetySystem::set_servo(int servo, int pulse) {
    static int last_pulse[18] = {0};

    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);

    if(pulse <= MIN_PULSE + 50 || pulse >= MAX_PULSE - 50) {
        Logger::log(Logger::ERROR, "CRITICAL servo %d: %dus", servo, pulse);
        emergency_stop();
        return false;
    }
    if (abs(pulse - last_pulse[servo]) < 10) {
        return false;
    }

    last_pulse[servo] = pulse;

    Commands::send_servo(servo, pulse);
    return true;
}

void SafetySystem::update_load_monitor() {
    if (millis() - last_update < 100) return;

    for (int i = 0; i < NUM_LEGS; i++) {
        current_load[i] = 0.8f * current_load[i] + 0.2f * read_current(i);

        if (current_load[i] > TORQUE_LIMIT) {
            Logger::log(Logger::WARNING, "Overload leg %d: %.1fA", i, current_load[i]);
            emergency_stop();
        }
    }
    last_update = millis();
}

void SafetySystem::emergency_stop() {
    for (int i = 0; i < NUM_LEGS * SERVOS_PER_LEG; i++) {
        Commands::send_servo(i, NEUTRAL);
    }
    Logger::log(Logger::ERROR, "EMERGENCY STOP");
}

void SafetySystem::set_speed(float speed) {
    max_speed = constrain(speed, 0.0f, MAX_SPEED);
}

float SafetySystem::read_current(int leg) {
    static float zero_offset[NUM_LEGS];
    static unsigned long last_sample = 0;
    if (millis() - last_sample < CURRENT_SAMPLE_TIME) return 0.0f;

    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += analogReadMilliVolts(A0 + leg) / 1000.0f;
    }
    last_sample = millis();
    return (sum * 0.1f - zero_offset[leg]) * 0.8f;
}
