#pragma once
#include "config.h"
#include "safety.h"
#include <math.h>
#define degrees(rad) ((rad)*180.0 / PI)

struct LegPosition {
    float x;  // Позиция по X
    float y;  // Позиция по Y
    float z;  // Позиция по Z
    uint16_t duration;
    float coxa_angle;   // Расчетный угол коксы
    float femur_angle;  // Расчетный угол фемура
    float tibia_angle;  // Расчетный угол тибии
};

class LegController {
public:
    void reset_pose(LegID leg_id) {
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int servo = LEG_SERVO_MAP[leg_id][joint];
            Commands::send_servo_direct(servo, NEUTRAL + LEG_OFFSETS[leg_id][joint]);
        }
    }

    // Остальные методы оставлены пустыми
    void update_all_legs() {}
    void set_target_position(LegID leg, const LegPosition& pos) {}
};
