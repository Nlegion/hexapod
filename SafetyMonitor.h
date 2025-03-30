#pragma once
#include <Arduino.h>

class SafetyMonitor {
private:
  struct ServoLimits {
    int min;
    int max;
    int maxSpeed;
  };

  ServoLimits limits[32];

public:
  SafetyMonitor();
  bool validate_command(int servo, int target, int speed);
};
