#include "safety.h"
#include "commands.h"

float SafetySystem::current_load[TOTAL_LEGS];
float SafetySystem::max_speed = MAX_SPEED;
unsigned long SafetySystem::last_update = 0;

inline int SafetySystem::get_speed() {
    return static_cast<int>(max_speed * 1000.0f / MAX_SPEED);
}

void SafetySystem::init() {
    for (int i = 0; i < TOTAL_LEGS; i++)
        current_load[i] = 0.0f;
    max_speed = MAX_SPEED;
    pinMode(A0, ANALOG);
}

bool SafetySystem::set_servo(int servo, int pulse) {
    Logger::log(Logger::INFO, "Set servo %d → %dμs", servo, pulse);
    static int last_pulse[32] = {0};
    const int MAX_DELTA = 15; // Уменьшено для плавности

    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
    
    if(pulse < MIN_PULSE || pulse > MAX_PULSE) {
        Logger::log(Logger::ERROR, 
            "Pulse out of range! Servo %d: %dμs", servo, pulse);
        return false;
    }
    int delta = pulse - last_pulse[servo];
    delta = constrain(delta, -MAX_DELTA, MAX_DELTA);
    pulse = last_pulse[servo] + delta;

    last_pulse[servo] = pulse;
    Commands::send_servo(servo, pulse);
    return true;
}

void SafetySystem::update_load_monitor() {
    if (millis() - last_update < 100) return;

    for (int i = 0; i < TOTAL_LEGS; i++) {
        current_load[i] = 0.8f * current_load[i] + 0.2f * read_current(i);

        if (current_load[i] > TORQUE_LIMIT) {
            Logger::log(Logger::WARNING, "Overload leg %d: %.1fA", i, current_load[i]);
            emergency_stop();
        }
    }
    last_update = millis();
}

void SafetySystem::emergency_stop() {
    for (int i = 0; i < TOTAL_LEGS * SERVOS_PER_LEG; i++) {
        Commands::send_servo(i, NEUTRAL);
    }
    Logger::log(Logger::ERROR, "EMERGENCY STOP");
}

void SafetySystem::set_speed(float speed) {
    max_speed = constrain(speed, 0.0f, MAX_SPEED);
}

float SafetySystem::read_current(int leg) {
    static float zero_offset[TOTAL_LEGS];
    static unsigned long last_sample = 0;
    if (millis() - last_sample < CURRENT_SAMPLE_TIME) return 0.0f;

    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += analogReadMilliVolts(A0 + leg) / 1000.0f;
    }
    last_sample = millis();
    return (sum * 0.1f - zero_offset[leg]) * 0.8f;
}
