#pragma once
#include "config.h"
#include "safety.h"
#include "logger.h"

// Результаты выполнения команд
enum class CommandResult {
  SUCCESS,
  INVALID_SERVO,
  INVALID_PULSE,
  SERIAL_ERROR,
  TIMEOUT
};

// Статус контроллера сервоприводов
class ControllerStatus {
private:
  static bool initialized;
  static unsigned long last_command_time;
  static int command_count;
  
public:
  static bool is_ready() { return initialized && Serial1; }
  static void mark_command_sent() { 
    last_command_time = millis(); 
    command_count++; 
  }
  static unsigned long time_since_last_command() { 
    return millis() - last_command_time; 
  }
  static int get_command_count() { return command_count; }
  static void set_initialized(bool state) { initialized = state; }
};

class Commands {
private:
  // Валидация параметров команд
  static bool validate_servo_channel(int servo) {
    return (servo >= 1 && servo <= 32);
  }
  
  static bool validate_pulse_value(int pulse) {
    return (pulse >= MIN_PULSE && pulse <= MAX_PULSE);
  }
  
  // Безопасная отправка команды с проверками
  static CommandResult send_command_safe(const char* command) {
    if (!ControllerStatus::is_ready()) {
      Logger::log(Logger::ERROR, "Controller not ready for command: %s", command);
      return CommandResult::SERIAL_ERROR;
    }
    
    // Проверяем, не отправляем ли команды слишком быстро
    if (ControllerStatus::time_since_last_command() < 1) {
      delayMicroseconds(500); // Минимальная задержка между командами
    }
    
    size_t bytes_written = Serial1.print(command);
    if (bytes_written == 0) {
      Logger::log(Logger::ERROR, "Failed to send command: %s", command);
      return CommandResult::SERIAL_ERROR;
    }
    
    ControllerStatus::mark_command_sent();
    Logger::log(Logger::DEBUG, "Command sent: %s", command);
    return CommandResult::SUCCESS;
  }

public:
  // Инициализация и очистка буфера контроллера
  static CommandResult init_controller() {
    Logger::log(Logger::INFO, "Initializing 32-channel servo controller");
    
    // Проверяем доступность Serial1
    if (!Serial1) {
      Logger::log(Logger::ERROR, "Serial1 not available for servo controller");
      return CommandResult::SERIAL_ERROR;
    }
    
    // Отправляем команды напрямую без проверки готовности (для инициализации)
    size_t bytes1 = Serial1.print("#255P0T0\r\n");    // Стоп всех каналов
    if (bytes1 == 0) {
      Logger::log(Logger::ERROR, "Failed to send stop command");
      return CommandResult::SERIAL_ERROR;
    }
    delay(100);
    
    size_t bytes2 = Serial1.print("#0P1500T0\r\n");  // Сброс в нейтральное положение
    if (bytes2 == 0) {
      Logger::log(Logger::ERROR, "Failed to send reset command");
      return CommandResult::SERIAL_ERROR;
    }
    delay(100);
    
    // Очищаем буферы Serial1
    int bytes_cleared = 0;
    while (Serial1.available()) {
      Serial1.read();
      bytes_cleared++;
    }
    if (bytes_cleared > 0) {
      Logger::log(Logger::INFO, "Cleared %d bytes from Serial1 buffer", bytes_cleared);
    }
    
    // Устанавливаем статус готовности контроллера
    ControllerStatus::set_initialized(true);
    
    // Дополнительная пауза для стабилизации контроллера
    delay(500);
    Logger::log(Logger::INFO, "Controller initialized and buffers cleared");
    return CommandResult::SUCCESS;
  }

  // Прямая отправка команды сервоприводу без SafetySystem
  static CommandResult send_servo_direct(int servo, int pulse) {
    // Валидация параметров
    if (!validate_servo_channel(servo)) {
      Logger::log(Logger::ERROR, "Invalid servo channel: %d (must be 1-32)", servo);
      return CommandResult::INVALID_SERVO;
    }
    
    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
    if (!validate_pulse_value(pulse)) {
      Logger::log(Logger::ERROR, "Invalid pulse value: %d (must be %d-%d)", pulse, MIN_PULSE, MAX_PULSE);
      return CommandResult::INVALID_PULSE;
    }

    char buffer[32];
    snprintf(buffer, sizeof(buffer), "#%dP%dT0\r\n", servo, pulse);
    
    CommandResult result = send_command_safe(buffer);
    if (result == CommandResult::SUCCESS) {
      delayMicroseconds(500);  // Задержка для гарантированной отправки
    }
    
    return result;
  }

  // Отправка команды через SafetySystem (рекомендуемый метод)
  static CommandResult send_servo(int servo, int pulse) {
    // Валидация параметров
    if (!validate_servo_channel(servo)) {
      Logger::log(Logger::ERROR, "Invalid servo channel: %d (must be 1-32)", servo);
      return CommandResult::INVALID_SERVO;
    }
    
    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
    if (!validate_pulse_value(pulse)) {
      Logger::log(Logger::ERROR, "Invalid pulse value: %d (must be %d-%d)", pulse, MIN_PULSE, MAX_PULSE);
      return CommandResult::INVALID_PULSE;
    }

    // Формат команды: #<ch>P<pulse>T<time>\r\n
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "#%dP%dT%d\r\n",
             servo, pulse, SafetySystem::get_speed());

    CommandResult result = send_command_safe(cmd);
    if (result == CommandResult::SUCCESS) {
      delayMicroseconds(100);  // Задержка для стабильности
    }
    
    return result;
  }
  static CommandResult calibration_mode() {
    Logger::log(Logger::INFO, "Starting calibration mode");
    const int calibration_order[NUM_JOINTS] = { COXA, FEMUR, TIBIA };
    const char* joint_names[] = {"COXA", "FEMUR", "TIBIA"};
    int error_count = 0;

    for (int joint : calibration_order) {
      Logger::log(Logger::INFO, "Testing %s joints", joint_names[joint]);
      for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        int servo = LEG_SERVO_MAP[leg][joint];
        int calib_pulse = (leg >= 3) ? (NEUTRAL - 400) : (NEUTRAL + 400);
        calib_pulse = constrain(calib_pulse, MIN_PULSE, MAX_PULSE);

        Logger::log(Logger::INFO, "Leg %d, Joint %s, Servo %d -> %d", 
                   leg, joint_names[joint], servo, calib_pulse);
        
        CommandResult result = send_servo_direct(servo, calib_pulse);
        if (result != CommandResult::SUCCESS) {
          Logger::log(Logger::ERROR, "Failed to calibrate servo %d, error: %d", servo, (int)result);
          error_count++;
        }
        delay(1500);
      }
      delay(2000); // Пауза между суставами
    }
    
    Logger::log(Logger::INFO, "Calibration complete, returning to neutral (errors: %d)", error_count);
    CommandResult reset_result = reset_all_servos();
    
    if (error_count > 0) {
      Logger::log(Logger::WARNING, "Calibration completed with %d errors", error_count);
    }
    
    return (error_count == 0 && reset_result == CommandResult::SUCCESS) ? 
           CommandResult::SUCCESS : CommandResult::SERIAL_ERROR;
  }

  static void test_single_leg(int leg_id) {
    if (leg_id < 0 || leg_id >= TOTAL_LEGS) return;
    
    Logger::log(Logger::INFO, "Testing leg %d SAFELY", leg_id);
    const char* joint_names[] = {"COXA", "FEMUR", "TIBIA"};
    
    // Безопасные пределы движения для тестирования
    const int SAFE_RANGE = 200; // Увеличено с 150 до 200 для лучшей видимости
    
    // Тест каждого сустава ноги
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
      int servo = LEG_SERVO_MAP[leg_id][joint];
      
      Logger::log(Logger::INFO, "Testing leg %d, joint %s (servo %d)", 
                 leg_id, joint_names[joint], servo);
      
      // Движение в одну сторону (с ограничением)
      int pulse1 = constrain(NEUTRAL + SAFE_RANGE, MIN_PULSE, MAX_PULSE);
      Commands::send_servo_direct(servo, pulse1);
      Logger::log(Logger::INFO, "Servo %d -> %d", servo, pulse1);
      delay(800);
      
      // Движение в другую сторону (с ограничением)
      int pulse2 = constrain(NEUTRAL - SAFE_RANGE, MIN_PULSE, MAX_PULSE);
      Commands::send_servo_direct(servo, pulse2);
      Logger::log(Logger::INFO, "Servo %d -> %d", servo, pulse2);
      delay(800);
      
      // Возврат в нейтраль с калибровочным смещением
      int neutral_pulse = constrain(NEUTRAL + LEG_OFFSETS[leg_id][joint], MIN_PULSE, MAX_PULSE);
      Commands::send_servo_direct(servo, neutral_pulse);
      Logger::log(Logger::INFO, "Servo %d -> neutral %d", servo, neutral_pulse);
      delay(500);
    }
    Logger::log(Logger::INFO, "Leg %d test completed", leg_id);
  }

  static void diagnostic_sequence() {
    Logger::log(Logger::INFO, "Starting SAFE diagnostic sequence");
    
    const int SAFE_DIAG_RANGE = 150; // Увеличено с 100 до 150 для лучшей видимости
    
    // 1. Проверка всех сервоприводов по очереди с безопасными пределами
    for (int i = 1; i <= 32; i++) {
      Logger::log(Logger::INFO, "Testing servo %d safely", i);
      
      int pulse_high = constrain(NEUTRAL + SAFE_DIAG_RANGE, MIN_PULSE, MAX_PULSE);
      Commands::send_servo_direct(i, pulse_high);
      delay(250);
      
      int pulse_low = constrain(NEUTRAL - SAFE_DIAG_RANGE, MIN_PULSE, MAX_PULSE);
      Commands::send_servo_direct(i, pulse_low);
      delay(250);
      
      Commands::send_servo_direct(i, NEUTRAL);
      delay(150);
    }
    
    Logger::log(Logger::INFO, "Safe diagnostic sequence completed");
    
    // Финальная проверка - сброс всех в стартовую позицию
    delay(1000);
    reset_all_servos();
  }

  static CommandResult reset_all_servos() {
    Logger::log(Logger::INFO, "Resetting all servos to startup position");
    int error_count = 0;
    
    // Используем ту же логику, что и при запуске
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      for (int joint = 0; joint < NUM_JOINTS; joint++) {
        int servo = LEG_SERVO_MAP[leg][joint];
        int startup_pulse = NEUTRAL + LEG_OFFSETS[leg][joint];
        startup_pulse = constrain(startup_pulse, MIN_PULSE, MAX_PULSE);
        
        CommandResult result = send_servo_direct(servo, startup_pulse);
        if (result != CommandResult::SUCCESS) {
          Logger::log(Logger::ERROR, "Failed to reset servo %d, error: %d", servo, (int)result);
          error_count++;
        } else {
          Logger::log(Logger::DEBUG, "Leg %d, Joint %d, Servo %d -> %d", 
                     leg, joint, servo, startup_pulse);
        }
      }
      delay(100); // Небольшая задержка между ногами
    }
    
    if (error_count > 0) {
      Logger::log(Logger::ERROR, "Reset completed with %d errors", error_count);
      return CommandResult::SERIAL_ERROR;
    }
    
    Logger::log(Logger::INFO, "Reset complete - all %d servos reset successfully", 
               TOTAL_LEGS * NUM_JOINTS);
    return CommandResult::SUCCESS;
  }
  
  // Получение статистики команд
  static void print_statistics() {
    Logger::log(Logger::INFO, "Command Statistics:");
    Logger::log(Logger::INFO, "  Controller ready: %s", ControllerStatus::is_ready() ? "YES" : "NO");
    Logger::log(Logger::INFO, "  Commands sent: %d", ControllerStatus::get_command_count());
    Logger::log(Logger::INFO, "  Time since last command: %lu ms", ControllerStatus::time_since_last_command());
  }
};

// Статические переменные ControllerStatus определены в hexapod.ino
