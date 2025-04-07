// === config.h === (исправленная версия)
#pragma once
#include <Arduino.h>

// Декларация типов
enum LegID {
    LEG_FRONT_RIGHT,    // 0
    LEG_MIDDLE_RIGHT,   // 1
    LEG_REAR_RIGHT,     // 2
    LEG_REAR_LEFT,      // 3
    LEG_MIDDLE_LEFT,    // 4
    LEG_FRONT_LEFT,     // 5
    TOTAL_LEGS // Количество ног
};

enum JointID {
    COXA,   // Сустав ближе к телу
    FEMUR,  // Бедренный сустав
    TIBIA,  // Голенный сустав
    NUM_JOINTS
};

// Конфигурация сервоприводов для каждой ноги [COXA, FEMUR, TIBIA]
constexpr uint8_t LEG_SERVO_MAP[TOTAL_LEGS][NUM_JOINTS] = {
    /* LEG_FRONT_RIGHT */ {9, 10, 11},
    /* LEG_MIDDLE_RIGHT */ {5, 6, 7},
    /* LEG_REAR_RIGHT */ {1, 2, 3},
    /* LEG_REAR_LEFT */ {32, 31, 30},
    /* LEG_MIDDLE_LEFT */ {28, 27, 26},
    /* LEG_FRONT_LEFT */ {21, 22, 23}
};

constexpr int LEG_OFFSETS[TOTAL_LEGS][NUM_JOINTS] = {
    /* LEG_FRONT_RIGHT */ { -15, 10, -5 }, // COX, FEMUR, TIBIA
    /* LEG_MIDDLE_RIGHT */ { 0, 0, 0 },
    /* LEG_REAR_RIGHT */ { 0, 0, 0 },
    /* LEG_REAR_LEFT */ { -30, 20, -10 },
    /* LEG_MIDDLE_LEFT */ { 0, 0, 0 },
    /* LEG_FRONT_LEFT */ { 0, 0, 0 }
};

// Network
constexpr char SSID[] = "Homenet_plus";
constexpr char PASSWORD[] = "29pronto69";
constexpr int WIFI_TIMEOUT = 20;

// Servo
constexpr int SERVOS_PER_LEG = 3;
constexpr int MIN_PULSE = 500;   // Минимальный импульс для MG90S
constexpr int MAX_PULSE = 2500;  // Максимальный импульс
constexpr int NEUTRAL = 1500;

// Kinematics
constexpr float BODY_RADIUS = 130.0f;
constexpr float FEMUR_LENGTH = 40.0f;
constexpr float TIBIA_LENGTH = 90.0f;
constexpr float MAX_STEP = 80.0f;
constexpr float STEP_DURATION = 4.0f;
constexpr float STEP_LENGTH = 50.0f;
constexpr float STEP_HEIGHT = 30.0f;
constexpr float MAX_ANGLES[3] = {90.0f, 90.0f, 90.0f};

// Safety
constexpr float MAX_SPEED = 50.0f;
constexpr float TORQUE_LIMIT = 2.0f;
constexpr float CURRENT_SAMPLE_TIME = 500;
