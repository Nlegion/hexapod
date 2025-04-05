#include "safety.h"
#include "commands.h"

float SafetySystem::current_load[NUM_LEGS];
float SafetySystem::max_speed = MAX_SPEED;
unsigned long SafetySystem::last_update = 0;

inline int SafetySystem::get_speed() {
    return static_cast<int>(max_speed * 1000.0f / MAX_SPEED);
}

void SafetySystem::init() {
    for (int i = 0; i < NUM_LEGS; i++)
        current_load[i] = 0.0f;
    max_speed = MAX_SPEED;
    pinMode(A0, ANALOG);
}

bool SafetySystem::set_servo(int servo, int pulse) {
    static int last_pulse[18] = {0};
    const int MAX_DELTA = 2000;

    pulse = constrain(pulse, MIN_PULSE + 50, MAX_PULSE - 50);

    int delta = abs(pulse - last_pulse[servo]);
    if(delta > 500) {
        pulse = last_pulse[servo] + (pulse > last_pulse[servo] ? 500 : -500);
        Logger::log(Logger::WARNING, "Servo %d speed limited", servo);
    }

    last_pulse[servo] = pulse;
    // Добавьте сюда код для отправки команды сервоприводу
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
