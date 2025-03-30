#include "SafetyMonitor.h"

SafetyMonitor::SafetyMonitor() {
  for (int i = 0; i < 32; i++) {
    limits[i] = { 500, 2500, 500 };
  }
}

bool SafetyMonitor::validate_command(int servo, int target, int speed) {
  return (target >= limits[servo].min) && (target <= limits[servo].max) && (speed <= limits[servo].maxSpeed);
}

SafetyMonitor safetyMonitor;
