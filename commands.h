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
    
    Logger::log(Logger::INFO, "Testing leg %d", leg_id);
    
    // Тест каждого сустава ноги
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
      int servo = LEG_SERVO_MAP[leg_id][joint];
      
      // Движение в одну сторону
      Commands::send_servo_direct(servo, NEUTRAL + 300);
      delay(1000);
      
      // Движение в другую сторону  
      Commands::send_servo_direct(servo, NEUTRAL - 300);
      delay(1000);
      
      // Возврат в нейтраль
      Commands::send_servo_direct(servo, NEUTRAL + LEG_OFFSETS[leg_id][joint]);
      delay(500);
    }
  }

  static void diagnostic_sequence() {
    Logger::log(Logger::INFO, "Starting diagnostic sequence");
    
    // 1. Проверка всех сервоприводов по очереди
    for (int i = 1; i <= 32; i++) {
      Logger::log(Logger::INFO, "Testing servo %d", i);
      Commands::send_servo_direct(i, NEUTRAL + 200);
      delay(300);
      Commands::send_servo_direct(i, NEUTRAL - 200);  
      delay(300);
      Commands::send_servo_direct(i, NEUTRAL);
      delay(200);
    }
    
    Logger::log(Logger::INFO, "All servos tested");
  }

  static void reset_all_servos() {
    Logger::log(Logger::INFO, "Resetting all servos to neutral + offsets");
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      for (int joint = 0; joint < NUM_JOINTS; joint++) {
        int servo = LEG_SERVO_MAP[leg][joint];
        Commands::send_servo_direct(servo, NEUTRAL + LEG_OFFSETS[leg][joint]);
      }
    }
  }
};
