// GaitController.h
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

private:
  void update_tripod_gait();
  void update_wave_gait();
  void update_ripple_gait();
  void update_metachromatic_gait();
  void calculate_leg_movement(int leg, float phase);

  float calculate_hip_angle(float x, float z);
  float calculate_knee_angle(float x, float z);
  float calculate_ankle_angle(float x, float z);
};

extern GaitController gaitController;

#endif
