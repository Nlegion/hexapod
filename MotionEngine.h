#pragma once
#include <Arduino.h>
#include "SafetyMonitor.h"
#include "ServoManager.h"

class MotionEngine {
private:
  enum class MotionState {
    IDLE,
    MOVING,
    CALIBRATING
  };

  MotionState currentState = MotionState::IDLE;
  unsigned long motionStartTime;
  SafetyMonitor safetyMonitor;
  ServoManager servoManager;

public:
  void start_movement(const vector<int>& targets, int duration) {
        currentState = MotionState::MOVING;
        motionStartTime = millis();
        
        // Применяем калибровку перед отправкой
        calibrationManager.applyCalibration(SAdj);

        for (size_t i = 0; i < targets.size(); i += 2) {
            int servo = targets[i];
            int target = targets[i + 1];
            int calibratedTarget = target + SAdj[servo]; // Добавляем калибровку
            
            if (safetyMonitor.validate_command(servo, calibratedTarget, StepSpeed)) {
                servoManager.set_position(servo, calibratedTarget);
            }
        }
        Send_Comm();
    }


  void update() {
    switch (currentState) {
      case MotionState::MOVING:
        if (millis() - motionStartTime >= 20) {  // 50 Hz
          currentState = MotionState::IDLE;
          Send_Comm();
        }
        break;
      case MotionState::CALIBRATING:
        // Процедура калибровки
        break;
    }
  }
};

extern MotionEngine motionEngine;
