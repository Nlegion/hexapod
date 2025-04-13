#pragma once
#include "config.h"
#include "safety.h"
#include <math.h>
#define degrees(rad) ((rad)*180.0 / PI)

struct LegPosition {
    float x;  // Позиция по X
    float y;  // Позиция по Y
    float z;  // Позиция по Z
    uint16_t duration; };

class LegController {
private:
    LegPosition target_pos[TOTAL_LEGS];
    LegPosition current_pos[TOTAL_LEGS];

    bool is_left_leg(LegID leg) {
        return LEG_ORIENTATION[leg] < 0;
    }

public:
    void set_target_position(LegID leg, const LegPosition& pos) {
        target_pos[leg] = pos;
    }

    void calculate_ik(LegID leg, float x, float y, float z, float angles[3]) {
        // Поворот системы координат согласно ориентации ноги
        float theta_body = radians(LEG_ORIENTATION[leg]);
        float rotated_x = x * cos(theta_body) - y * sin(theta_body);
        float rotated_y = x * sin(theta_body) + y * cos(theta_body);
        
        // Параметры геометрии
        float coxa_length = BODY_RADIUS;
        float L = sqrt(rotated_x*rotated_x + rotated_y*rotated_y) - coxa_length;
        float D = sqrt(L*L + z*z);

        // Проверка достижимости
        if(D > FEMUR_LENGTH + TIBIA_LENGTH || D < fabs(FEMUR_LENGTH - TIBIA_LENGTH)) {
            Logger::log(Logger::ERROR, "Unreachable position for leg %d: (%.1f, %.1f, %.1f)", 
                      leg, x, y, z);
            angles[COXA] = angles[FEMUR] = angles[TIBIA] = 0.0f;
            return;
        }

        // Расчет углов
        angles[COXA] = atan2(rotated_y, rotated_x);
        
        float theta = atan2(z, L);
        float alpha = acos((FEMUR_LENGTH*FEMUR_LENGTH + D*D - TIBIA_LENGTH*TIBIA_LENGTH) 
                      / (2*FEMUR_LENGTH*D));
        
        angles[FEMUR] = theta + alpha;
        angles[TIBIA] = PI - acos((FEMUR_LENGTH*FEMUR_LENGTH + TIBIA_LENGTH*TIBIA_LENGTH - D*D) 
                      / (2*FEMUR_LENGTH*TIBIA_LENGTH));

        // Применение ограничений
        angles[COXA] = constrain(angles[COXA], 
                              radians(ANGLE_LIMITS[COXA][0]), 
                              radians(ANGLE_LIMITS[COXA][1]));
        
        angles[FEMUR] = constrain(angles[FEMUR], 
                               radians(ANGLE_LIMITS[FEMUR][0]), 
                               radians(ANGLE_LIMITS[FEMUR][1]));
        
        angles[TIBIA] = constrain(angles[TIBIA], 
                               radians(ANGLE_LIMITS[TIBIA][0]), 
                               radians(ANGLE_LIMITS[TIBIA][1]));
        Logger::log(Logger::INFO, 
        "Leg %d IK: X=%.1f Y=%.1f Z=%.1f → COX=%.1f FEM=%.1f TIB=%.1f",
        leg, x, y, z, 
        degrees(angles[COXA]),
        degrees(angles[FEMUR]),
        degrees(angles[TIBIA]));
    
    }

     void reset_pose(LegID leg_id) {
    const char* JOINT_NAMES[] = {"COXA", "FEMUR", "TIBIA"};
    
    for(int joint = 0; joint < NUM_JOINTS; joint++) {
        int servo = LEG_SERVO_MAP[leg_id][joint];
        Logger::log(Logger::INFO, "Resetting %s leg %d → servo %d", 
                  JOINT_NAMES[joint], leg_id, servo);
        Commands::send_servo_direct(servo, NEUTRAL);
    }
    current_pos[leg_id] = {0, 0, 0, 0};
}

    void update_all_legs(float progress) {
    for(int leg = 0; leg < TOTAL_LEGS; leg++) {
        current_pos[leg].x = current_pos[leg].x + (target_pos[leg].x - current_pos[leg].x) * 0.3f;
        current_pos[leg].y = current_pos[leg].y + (target_pos[leg].y - current_pos[leg].y) * 0.3f;
        current_pos[leg].z = current_pos[leg].z + (target_pos[leg].z - current_pos[leg].z) * 0.3f;
        
        apply_angles(static_cast<LegID>(leg), current_pos[leg]);
    }
}

float lerp(float a, float b, float t) {
    return a + t * (b - a);
}


private:
    void apply_angles(LegID leg, const LegPosition& pos) {
    float angles[NUM_JOINTS];
    calculate_ik(leg, pos.x, pos.y, pos.z, angles);

    int direction = is_left_leg(leg) ? -1 : 1;
    
    for(int j = 0; j < NUM_JOINTS; j++) {
        int servo = LEG_SERVO_MAP[leg][j];
        float deg = degrees(angles[j]);
        
        // Корректировка коэффициента пересчёта градусов в импульсы
        // В файле kinematics.h изменить формулу расчёта импульсов:
int pulse = NEUTRAL + (LEG_OFFSETS[leg][j] + deg * direction) * (2000.0f / 180.0f); // 11.11 мкс/градус → ~11.11
pulse = constrain(pulse, MIN_PULSE + 100, MAX_PULSE - 100); // Добавить защитный интервал
        
        SafetySystem::set_servo(servo, pulse);
    }
}

};
