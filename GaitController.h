#ifndef GAITCONTROLLER_H
#define GAITCONTROLLER_H

#include <Arduino.h>
#include "ServoManager.h"

enum class GaitType {
  TRIPOD,
  WAVE,
  RIPPLE,
  METACHROMATIC
};

class GaitController {
private:
  GaitType currentGait = GaitType::TRIPOD;
  float phase = 0.0f;
  float speed = 1.0f;

public:
  void update();
  void set_gait(GaitType type);
  void set_speed(float newSpeed) {
      speed = constrain(newSpeed, 0.5f, 3.0f);
  }

private:
  void update_tripod_gait();
  void calculate_leg_movement(int leg, float phase);
};

extern GaitController gaitController;

#endif
