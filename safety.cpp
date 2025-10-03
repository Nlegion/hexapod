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
  const int MAX_DELTA = 120;  // Улучшенная плавность движений (было 200)
  static int last_pulse[32] = { 0 };
  static bool initialized = false;
  
  // Инициализируем last_pulse нейтральными значениями при первом вызове
  if (!initialized) {
    for (int i = 0; i < 32; i++) {
      last_pulse[i] = NEUTRAL;
    }
    initialized = true;
  }

  // Проверяем валидность канала сервопривода
  if (servo < 1 || servo > 32) {
    Logger::log(Logger::ERROR, "Invalid servo channel: %d", servo);
    return false;
  }
  
  pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
  int delta = pulse - last_pulse[servo - 1];  // last_pulse индексируется с 0

  // Адаптивное ограничение скорости - более плавное для малых изменений
  int adaptive_delta = MAX_DELTA;
  if (abs(delta) < 50) {
    adaptive_delta = MAX_DELTA / 2; // Очень плавно для малых движений
  }

  if (abs(delta) > adaptive_delta) {
    pulse = last_pulse[servo - 1] + (delta > 0 ? adaptive_delta : -adaptive_delta);
    Logger::log(Logger::DEBUG,
                "Servo %d adaptive speed limited: %d → %d (delta: %d, limit: %d)",
                servo, last_pulse[servo - 1], pulse, delta, adaptive_delta);
  }

  Commands::send_servo(servo, pulse);
  last_pulse[servo - 1] = pulse;
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
  Logger::log(Logger::ERROR, "EMERGENCY STOP - Setting all servos to neutral");
  
  // Отправляем команду сброса всех сервоприводов в нейтральное положение
  // Используем прямую отправку для скорости
  for (int servo = 1; servo <= 32; servo++) {
    Commands::send_servo_direct(servo, NEUTRAL);
  }
  
  // Дополнительно отправляем глобальную команду остановки контроллера
  Serial1.print("#0P0T0\r\n"); // Команда остановки всех сервоприводов
  delay(100);
  
  Logger::log(Logger::ERROR, "EMERGENCY STOP COMPLETED - All servos stopped");
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
