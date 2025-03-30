#include "GaitController.h"
#include <math.h>
#include "config.h"

GaitController gaitController;

void GaitController::update() {
  // Обновляем фазу анимации
  phase += 0.1f * speed;
  if (phase >= 2 * M_PI) phase -= 2 * M_PI;

  // Выбираем соответствующую походку
  switch (currentGait) {
    case GaitType::TRIPOD:
      update_tripod_gait();
      break;
    case GaitType::WAVE:
      update_wave_gait();
      break;
    case GaitType::RIPPLE:
      update_ripple_gait();
      break;
    case GaitType::METACHROMATIC:
      update_metachromatic_gait();
      break;
  }
}

void GaitController::update_tripod_gait() {
  for (int i = 0; i < 6; i++) {
    float legPhase = phase + (i % 2) * M_PI;
    calculate_leg_movement(i, legPhase);
  }
}

void GaitController::update_wave_gait() {
  for (int i = 0; i < 6; i++) {
    float legPhase = phase + i * (2 * M_PI / 6);
    calculate_leg_movement(i, legPhase);
  }
}

void GaitController::update_ripple_gait() {
  for (int i = 0; i < 6; i++) {
    float legPhase = phase + i * (M_PI / 3);
    calculate_leg_movement(i, legPhase);
  }
}

void GaitController::update_metachromatic_gait() {
  for (int i = 0; i < 6; i++) {
    float legPhase = phase + i * (M_PI / 2);
    calculate_leg_movement(i, legPhase);
  }
}

void GaitController::set_gait(GaitType type) {
  currentGait = type;
  phase = 0.0f; // Сбрасываем фазу при смене походки
}

void GaitController::calculate_leg_movement(int leg, float phase) {
  constexpr float STEP_HEIGHT = 50.0f;
  constexpr float STEP_LENGTH = 100.0f;

  float angle = phase;
  float x = STEP_LENGTH * cos(angle);
  float z = STEP_HEIGHT * sin(angle);

  // Инвертируем координаты для правой стороны
  if (isMirroredLeg(leg)) {
    x = -x;
  }

  int hipPos = calculatePosition(leg, calculate_hip_angle(x, z));
  int kneePos = calculatePosition(leg, calculate_knee_angle(x, z));
  int anklePos = calculatePosition(leg, calculate_ankle_angle(x, z));

  servoManager.set_position(leg * 3, hipPos);
  servoManager.set_position(leg * 3 + 1, kneePos);
  servoManager.set_position(leg * 3 + 2, anklePos);
}

float GaitController::calculate_hip_angle(float x, float z) {
  // Ваша реализация
  return 0.0f;
}

float GaitController::calculate_knee_angle(float x, float z) {
  // Ваша реализация
  return 0.0f;
}

float GaitController::calculate_ankle_angle(float x, float z) {
  // Ваша реализация
  return 0.0f;
}
