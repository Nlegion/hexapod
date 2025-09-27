#include "safety.h"
#include "commands.h"

float SafetySystem::current_load[TOTAL_LEGS];
float SafetySystem::max_speed = MAX_SPEED;
unsigned long SafetySystem::last_update = 0;

int SafetySystem::get_speed() {
  return static_cast<int>(max_speed * 1000.0f / MAX_SPEED);
}

void SafetySystem::init() {
  for (int i = 0; i < TOTAL_LEGS; i++)
    current_load[i] = 0.0f;
  max_speed = MAX_SPEED;
  pinMode(A0, ANALOG);
}

bool SafetySystem::set_servo(int servo, int pulse) {
  const int MAX_DELTA = 200;  // Более плавное движение
  static int last_pulse[32] = { 0 };

  pulse = constrain(pulse, MIN_PULSE + 100, MAX_PULSE - 100);
  int delta = pulse - last_pulse[servo];

  if (abs(delta) > MAX_DELTA) {
    pulse = last_pulse[servo] + (delta > 0 ? MAX_DELTA : -MAX_DELTA);
    Logger::log(Logger::WARNING,
                "Servo %d speed limited: %d → %d",
                servo, last_pulse[servo], pulse);
  }

  Commands::send_servo(servo, pulse);
  last_pulse[servo] = pulse;
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
