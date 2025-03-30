#include "GaitController.h"
#include <math.h>
#include "config.h"
#include "CalibrationManager.h"
extern int SAdj[32];

// Константы движения
constexpr float STEP_LENGTH = 80.0f;    // Длина шага в мм
constexpr float STEP_HEIGHT = 40.0f;    // Высота подъема ноги
constexpr float BODY_HEIGHT = 120.0f;   // Высота корпуса
constexpr float LEG_L1 = 50.0f;         // Длина бедра
constexpr float LEG_L2 = 70.0f;         // Длина голени

GaitController gaitController;

void GaitController::update() {
    phase += 0.05f * speed;
    if (phase >= 2 * M_PI) phase -= 2 * M_PI;

    switch (currentGait) {
        case GaitType::TRIPOD:
            update_tripod_gait();
            break;
        // Добавьте другие типы походок
    }
}

void GaitController::set_gait(GaitType type) {
    currentGait = type;
    phase = 0.0f; // Сброс фазы при смене походки
}

void GaitController::update_tripod_gait() {
    // Группы ног для триподной походки
    const int tripodGroups[2][3] = {{0, 3, 4}, {1, 2, 5}};
    int groupIndex = (phase < M_PI) ? 0 : 1;

    for (int i = 0; i < 3; i++) {
        int leg = tripodGroups[groupIndex][i];
        float legPhase = (phase < M_PI) ? phase : phase - M_PI;
        calculate_leg_movement(leg, legPhase);
    }
}

void GaitController::calculate_leg_movement(int leg, float phase) {
    // Траектория движения ноги (эллипс)
    float x = STEP_LENGTH * cos(phase);
    float z = STEP_HEIGHT * sin(phase) - BODY_HEIGHT;

    // Инвертируем для правой стороны
    if (isMirroredLeg(leg)) x = -x;

    // Кинематика обратная для двухзвенной ноги
    float d = sqrt(x*x + z*z);
    float alpha = atan2(z, x);
    float beta = acos((LEG_L1*LEG_L1 + d*d - LEG_L2*LEG_L2) / (2*LEG_L1*d));

    float hip_angle = (alpha + beta) * 180/M_PI;
    float knee_angle = acos((LEG_L1*LEG_L1 + LEG_L2*LEG_L2 - d*d) /
                          (2*LEG_L1*LEG_L2)) * 180/M_PI;

    // Преобразование углов в импульсы
    int hipPos = map(hip_angle, 0, 180, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
    int kneePos = map(knee_angle, 0, 180, MIN_SERVO_PULSE, MAX_SERVO_PULSE);

    // Применяем калибровку
    int legBase = leg * 3;
    servoManager.set_position(legBase, hipPos + SAdj[legBase]);
    servoManager.set_position(legBase+1, kneePos + SAdj[legBase+1]);
    servoManager.set_position(legBase+2, DEFAULT_SERVO_POS + SAdj[legBase+2]);
}
