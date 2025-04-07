#pragma once
#include "config.h"
#include "safety.h"
#include <math.h>
#define degrees(rad) ((rad) * 180.0 / PI)


constexpr float LEG_ORIENTATION[TOTAL_LEGS] = {
    0.0f,    // LEG_FRONT_RIGHT
    60.0f,   // LEG_MIDDLE_RIGHT
    120.0f,  // LEG_REAR_RIGHT
    -120.0f, // LEG_REAR_LEFT
    -60.0f,  // LEG_MIDDLE_LEFT
    0.0f     // LEG_FRONT_LEFT
};

class LegController {
public:
void reset_pose(LegID leg_id) {
        Logger::log(Logger::INFO, "Resetting leg %d", leg_id);
        for(int joint = 0; joint < NUM_JOINTS; joint++) {
            int servo = LEG_SERVO_MAP[leg_id][joint];
            int pulse = NEUTRAL + LEG_OFFSETS[leg_id][joint];
            pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
            SafetySystem::set_servo(servo, pulse);
        }
    }

 void update_single_leg(LegID leg_id, unsigned long elapsed_time) {
    float t = (float)elapsed_time / (STEP_DURATION * 1000); // t ∈ [0, 1]
    t = constrain(t, 0.0f, 1.0f);

    // Плавная интерполяция с использованием синусоидальной функции
    float smooth_t = 0.5f * (1.0f - cos(t * PI));

    float x = BODY_RADIUS + STEP_LENGTH * (smooth_t < 0.5 ? 2 * smooth_t : 2 * (1 - smooth_t));
    float z = -TIBIA_LENGTH + 20.0f + STEP_HEIGHT * sin(smooth_t * PI);

    static float prev_angles[NUM_JOINTS] = {0};
    float target_angles[NUM_JOINTS] = {0};
    calculate_ik(leg_id, x, 0, z, target_angles);

    // Интерполяция углов с учетом плавности
    float angles[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; j++) {
        angles[j] = prev_angles[j] + (target_angles[j] - prev_angles[j]) * smooth_t;
    }
    memcpy(prev_angles, angles, sizeof(angles));

    // Замедление отправки команд
    static unsigned long last_update = 0;
    if (millis() - last_update < 50) return; // 20 Hz (50 ms)
    last_update = millis();

    // Отправка импульсов
    for (int j = 0; j < NUM_JOINTS; j++) {
        int servo = LEG_SERVO_MAP[leg_id][j];
        float deg = degrees(angles[j]);
        int pulse = NEUTRAL + LEG_OFFSETS[leg_id][j] + deg * 11.11f;
        pulse = constrain(pulse, MIN_PULSE, MAX_PULSE); // Явное ограничение

        // Отладка углов
        Logger::log(Logger::INFO, "Leg %d, joint %d: deg=%.1f, offset=%d, pulse=%d",
                    leg_id, j, deg, LEG_OFFSETS[leg_id][j], pulse);

        SafetySystem::set_servo(servo, pulse);
    }
}


   void calculate_ik(int leg, float x, float y, float z, float angles[3]) {
    angles[0] = angles[1] = angles[2] = 0.0f;

    if (x < 50.0f || x > 200.0f || z < -100.0f) {
        Logger::log(Logger::ERROR, "Invalid target for leg %d: (%.1f, %.1f, %.1f)", leg, x, y, z);
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
    angles[COXA] = atan2(y, x) + radians(LEG_ORIENTATION[leg]);
    float theta = atan2(z, L);
    float alpha = acos((FEMUR_LENGTH*FEMUR_LENGTH + D*D - TIBIA_LENGTH*TIBIA_LENGTH)
                 / (2*FEMUR_LENGTH*D));
    angles[FEMUR] = theta + alpha;
    angles[TIBIA] = acos((FEMUR_LENGTH*FEMUR_LENGTH + TIBIA_LENGTH*TIBIA_LENGTH - D*D)
                 / (2*FEMUR_LENGTH*TIBIA_LENGTH));

    // Ограничение углов перед преобразованием в импульсы
    angles[COXA] = constrain(angles[COXA], -MAX_ANGLES[0], MAX_ANGLES[0]);
    angles[FEMUR] = constrain(angles[FEMUR], -MAX_ANGLES[1], MAX_ANGLES[1]);
    angles[TIBIA] = constrain(angles[TIBIA], -MAX_ANGLES[2], MAX_ANGLES[2]);

    // Преобразование радиан в градусы
    float deg_coxa = degrees(angles[COXA]);
    float deg_femur = degrees(angles[FEMUR]);
    float deg_tibia = degrees(angles[TIBIA]);

    // Логирование для отладки
    Logger::log(Logger::INFO, "IK Angles: %.1f, %.1f, %.1f", deg_coxa, deg_femur, deg_tibia);

    // Жесткое ограничение импульсов
    int pulse_coxa = NEUTRAL + LEG_OFFSETS[leg][COXA] + deg_coxa * 11.11f;
    int pulse_femur = NEUTRAL + LEG_OFFSETS[leg][FEMUR] + deg_femur * 11.11f;
    int pulse_tibia = NEUTRAL + LEG_OFFSETS[leg][TIBIA] + deg_tibia * 11.11f;

    if (pulse_coxa < MIN_PULSE || pulse_coxa > MAX_PULSE ||
        pulse_femur < MIN_PULSE || pulse_femur > MAX_PULSE ||
        pulse_tibia < MIN_PULSE || pulse_tibia > MAX_PULSE)
    {
        Logger::log(Logger::ERROR, "Target unreachable! leg=%d Pulses: %d, %d, %d",
                  leg, pulse_coxa, pulse_femur, pulse_tibia);
        angles[COXA] = angles[FEMUR] = angles[TIBIA] = 0.0f; // Сброс углов
    }
}


};
