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
        
        delayMicroseconds(500); // Задержка для гарантированной отправки
    }
    
    static void send_servo(int servo, int pulse) {
        pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
        if(servo < 0 || servo >= 32) return;

        // Формат команды: #<ch>P<pulse>T<time>\r\n
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "#%dP%dT%d\r\n", 
            servo, pulse, SafetySystem::get_speed());
        
        Serial1.print(cmd);
        delayMicroseconds(100); // Задержка для стабильности
    }
  static void calibration_mode() {
      const int calibration_order[NUM_JOINTS] = {COXA, FEMUR, TIBIA}; // Порядок калибровки
      
      for(int joint : calibration_order) {
          for(int leg = 0; leg < TOTAL_LEGS; leg++) {
              int servo = LEG_SERVO_MAP[leg][joint];
              int calib_pulse = (leg >= 3) ? 2000 : 1000; // Инверсия для левой стороны
              
              Commands::send_servo_direct(servo, calib_pulse);
              Logger::log(Logger::INFO, "Calib Leg %d Joint %d → %dμs", 
                        leg, joint, calib_pulse);
              delay(1000);
          }
      }
  }
};
