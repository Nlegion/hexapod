#pragma once
#include "config.h"
#include "safety.h"
#include "logger.h"

class Commands {
public:
  static void send_servo_direct(int servo, int pulse) {
    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);

    char buffer[32];
    snprintf(buffer, sizeof(buffer), "#%dP%dT0\r\n", servo, pulse);
    Serial1.print(buffer);

    delayMicroseconds(500);  // Задержка для гарантированной отправки
  }

  static void send_servo(int servo, int pulse) {
    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
    if (servo < 0 || servo >= 32) return;

    // Формат команды: #<ch>P<pulse>T<time>\r\n
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "#%dP%dT%d\r\n",
             servo, pulse, SafetySystem::get_speed());

    Serial1.print(cmd);
    delayMicroseconds(100);  // Задержка для стабильности
  }
  static void calibration_mode() {
    Logger::log(Logger::INFO, "Starting calibration mode");
    const int calibration_order[NUM_JOINTS] = { COXA, FEMUR, TIBIA };
    const char* joint_names[] = {"COXA", "FEMUR", "TIBIA"};

    for (int joint : calibration_order) {
      Logger::log(Logger::INFO, "Testing %s joints", joint_names[joint]);
      for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        int servo = LEG_SERVO_MAP[leg][joint];
        int calib_pulse = (leg >= 3) ? (NEUTRAL - 400) : (NEUTRAL + 400);
        calib_pulse = constrain(calib_pulse, MIN_PULSE, MAX_PULSE);

        Logger::log(Logger::INFO, "Leg %d, Joint %s, Servo %d -> %d", 
                   leg, joint_names[joint], servo, calib_pulse);
        Commands::send_servo_direct(servo, calib_pulse);
        delay(1500);
      }
      delay(2000); // Пауза между суставами
    }
    Logger::log(Logger::INFO, "Calibration complete, returning to neutral");
    reset_all_servos();
  }

  static void test_single_leg(int leg_id) {
    if (leg_id < 0 || leg_id >= TOTAL_LEGS) return;
    
    Logger::log(Logger::INFO, "Testing leg %d SAFELY", leg_id);
    const char* joint_names[] = {"COXA", "FEMUR", "TIBIA"};
    
    // Безопасные пределы движения для тестирования
    const int SAFE_RANGE = 150; // Уменьшено с 300 до 150
    
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
    
    const int SAFE_DIAG_RANGE = 100; // Безопасный диапазон для диагностики
    
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

  static void reset_all_servos() {
    Logger::log(Logger::INFO, "Resetting all servos to startup position");
    // Используем ту же логику, что и при запуске
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      for (int joint = 0; joint < NUM_JOINTS; joint++) {
        int servo = LEG_SERVO_MAP[leg][joint];
        int startup_pulse = NEUTRAL + LEG_OFFSETS[leg][joint];
        startup_pulse = constrain(startup_pulse, MIN_PULSE, MAX_PULSE);
        Commands::send_servo_direct(servo, startup_pulse);
        Logger::log(Logger::INFO, "Leg %d, Joint %d, Servo %d -> %d", 
                   leg, joint, servo, startup_pulse);
      }
      delay(100); // Небольшая задержка между ногами
    }
    Logger::log(Logger::INFO, "Reset complete");
  }
};
