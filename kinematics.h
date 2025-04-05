#pragma once
#include "config.h"
#include "safety.h"

class LegController {
private:
    // Добавляем структуру для хранения состояния анимации
    struct StepAnimation {
        float progress = 0.0f;
        bool is_animating = false;
    } step_anim;

public:
    // Объявляем метод calculate_ik в public
    void calculate_ik(int leg, float x, float y, float z, float angles[3]) {
        // Реализация метода
        // Пример: вычисление обратной кинематики для ноги
        // Этот код нужно заменить на реальную реализацию
        angles[0] = 0.0f; // Пример значения угла
        angles[1] = 0.0f; // Пример значения угла
        angles[2] = 0.0f; // Пример значения угла
    }

    void reset_pose() {
        Logger::log(Logger::INFO, "Resetting to neutral pose");
        
        for(int leg = 0; leg < NUM_LEGS; leg++) {
            for(int joint = 0; joint < 3; joint++) {
                int servo = leg * 3 + joint;
                int pulse = NEUTRAL + LEG_OFFSETS[leg][joint];
                
                pulse = constrain(pulse, MIN_PULSE + 100, MAX_PULSE - 100);
                
                if(SafetySystem::set_servo(servo, pulse)) {
                    Logger::log(Logger::INFO, "Set servo %d: %dus (leg %d joint %d)", 
                              servo, pulse, leg, joint);
                }
            }
        }
    }

    void update_single_leg(int leg_number, unsigned long elapsed_time) {
        float t = constrain((float)elapsed_time / (STEP_DURATION * 1000), 0.0f, 1.0f);

        // Траектория движения ноги (параболическая)
        float x = BODY_RADIUS + STEP_LENGTH * (t < 0.5 ? 2*t : 2*(1 - t));
        float z_base = -TIBIA_LENGTH + 20.0f;
        float z = z_base + STEP_HEIGHT * sin(t * PI);

        float angles[3] = {0};
        calculate_ik(leg_number, x, 0, z, angles);

        // Применяем углы только к выбранной ноге
        for(int j = 0; j < 3; j++) {
            int servo = leg_number * 3 + j;
            float deg = degrees(angles[j]);
            int pulse = NEUTRAL + LEG_OFFSETS[leg_number][j] + deg * 11.11f;

            SafetySystem::set_servo(servo, pulse);
        }

        Logger::log(Logger::INFO, "Leg %d: X=%.1f Z=%.1f Angles: %.1f, %.1f, %.1f",
                  leg_number, x, z,
                  degrees(angles[0]),
                  degrees(angles[1]),
                  degrees(angles[2]));
    }
};
