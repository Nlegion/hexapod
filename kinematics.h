#pragma once
#include "config.h"
#include "safety.h"

class LegController {
public:
    void reset_pose(LegID leg_id) {
    Logger::log(Logger::INFO, "Resetting leg %d", leg_id);
    for(int joint = 0; joint < NUM_JOINTS; joint++) {
        int servo = LEG_SERVO_MAP[leg_id][joint];
        int pulse = NEUTRAL + LEG_OFFSETS[leg_id][joint];
        pulse = constrain(pulse, MIN_PULSE, MAX_PULSE); // Добавьте ограничение
        SafetySystem::set_servo(servo, pulse);
    }
}

  void update_single_leg(LegID leg_id, unsigned long elapsed_time) {
    float t = (float)elapsed_time / (STEP_DURATION * 1000);
    t = constrain(t, 0.0f, 1.0f);

    float x = BODY_RADIUS + STEP_LENGTH * (t < 0.5 ? 2 * t : 2 * (1 - t));
    float z_base = -TIBIA_LENGTH + 20.0f;
    float z = z_base + STEP_HEIGHT * sin(t * PI);

    static float prev_angles[NUM_JOINTS] = {0};
    float target_angles[NUM_JOINTS] = {0};
    calculate_ik(leg_id, x, 0, z, target_angles);

    float angles[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; j++) {
        angles[j] = prev_angles[j] + (target_angles[j] - prev_angles[j]) * t; // Линейная интерполяция
    }
    memcpy(prev_angles, angles, sizeof(angles)); // Сохраняем текущие углы

    for (int j = 0; j < NUM_JOINTS; j++) {
        int servo = LEG_SERVO_MAP[leg_id][j];
        float deg = degrees(angles[j]);
        int pulse = NEUTRAL + LEG_OFFSETS[leg_id][j] + deg * 11.11f;

        SafetySystem::set_servo(servo, pulse);
    }

    Logger::log(Logger::INFO, "Leg %d: X=%.1f Z=%.1f Angles: %.1f, %.1f, %.1f",
                leg_id, x, z,
                degrees(angles[COXA]),
                degrees(angles[FEMUR]),
                degrees(angles[TIBIA]));
}


    void calculate_ik(int leg, float x, float y, float z, float angles[3]) {
    angles[0] = angles[1] = angles[2] = 0.0f;

    if (x < 0 || y < 0 || z < -TIBIA_LENGTH) {
        Logger::log(Logger::ERROR, "Invalid target position for leg %d", leg);
        return;
    }

    float coxa_length = BODY_RADIUS;
    float L = sqrt(x*x + y*y) - coxa_length;
    float D = sqrt(L*L + z*z);

    if(D > FEMUR_LENGTH + TIBIA_LENGTH || D < abs(FEMUR_LENGTH - TIBIA_LENGTH)) {
        Logger::log(Logger::ERROR, "Target unreachable! leg=%d (%.1f, %.1f, %.1f)",
                  leg, x, y, z);
        return;
    }

    // Реальные расчеты углов
    angles[COXA] = atan2(y, x); // Добавлен расчет угла COXA
    float theta = atan2(z, L);
    float alpha = acos((FEMUR_LENGTH*FEMUR_LENGTH + D*D - TIBIA_LENGTH*TIBIA_LENGTH)
                 / (2*FEMUR_LENGTH*D));
    angles[FEMUR] = theta + alpha;
    angles[TIBIA] = acos((FEMUR_LENGTH*FEMUR_LENGTH + TIBIA_LENGTH*TIBIA_LENGTH - D*D)
                  / (2*FEMUR_LENGTH*TIBIA_LENGTH));

    // Применяем ограничения
    angles[COXA] = constrain(angles[COXA], -MAX_ANGLES[0], MAX_ANGLES[0]);
    angles[FEMUR] = constrain(angles[FEMUR], -MAX_ANGLES[1], MAX_ANGLES[1]);
    angles[TIBIA] = constrain(angles[TIBIA], -MAX_ANGLES[2], MAX_ANGLES[2]);
}
};
