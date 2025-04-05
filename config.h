// === config.h ===
#pragma once
#define PI 3.14159265358979323846f

// Network
const char* SSID = "Homenet_plus";
const char* PASSWORD = "29pronto69";
const int WIFI_TIMEOUT = 20;

// Servo
constexpr int NUM_LEGS = 6;
constexpr int SERVOS_PER_LEG = 3;
constexpr int MIN_PULSE = 600;   // Минимальный импульс для MG90S
constexpr int MAX_PULSE = 2400;  // Максимальный импульс
constexpr int NEUTRAL = 1500;

// Kinematics
constexpr float BODY_RADIUS = 130.0f;
constexpr float FEMUR_LENGTH = 40.0f;
constexpr float TIBIA_LENGTH = 90.0f;
constexpr float STEP_HEIGHT = 40.0f;
constexpr float MAX_STEP = 80.0f;

// Safety
constexpr float MAX_SPEED = 100.0f; // mm/s
constexpr float TORQUE_LIMIT = 2.0f; // Повышаем порог срабатывания
constexpr float CURRENT_SAMPLE_TIME = 500; // Увеличиваем интервал измерения

#pragma once

// Смещения для калибровки сервоприводов (в микросекундах)
// Формат: {смещение_сустава1, смещение_сустава2, смещение_сустава3}
constexpr int LEG_OFFSETS[NUM_LEGS][3] = {
    // Передние правые ноги (0-2)
    { -10,  -30,   50 },
    {   0,  -25,   45 },
    { +15,  -20,   40 },
    
    // Задние левые ноги (3-5)
    {  -5,  +25,  -35 },
    { +10,  +30,  -40 },
    { -20,  +35,  -45 }
};
