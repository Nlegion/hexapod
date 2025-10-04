#pragma once
#include "Types.h"

// ═══════════════════════════════════════════════════════════════
// HARDWARE CONFIGURATION - Константы конфигурации hardware
// ═══════════════════════════════════════════════════════════════

namespace Core {
namespace Config {

// ═══════════════════════════════════════════════════════════════
// SERVO SYSTEM CONFIGURATION
// ═══════════════════════════════════════════════════════════════

constexpr int MIN_PULSE = 1000;     // Минимальный pulse (микросекунды)
constexpr int MAX_PULSE = 2000;     // Максимальный pulse
constexpr int NEUTRAL = 1500;       // Нейтральная позиция
constexpr int DEFAULT_TIME = 200;   // Время перехода по умолчанию (мс)

// Mapping: Нога -> [COXA, FEMUR, TIBIA] каналы
constexpr uint8_t LEG_SERVO_MAP[TOTAL_LEGS][NUM_JOINTS] = {
    /* LEG_FRONT_RIGHT  (0) */ { 9, 10, 11 },
    /* LEG_MIDDLE_RIGHT (1) */ { 5, 6, 7 },
    /* LEG_REAR_RIGHT   (2) */ { 1, 2, 3 },
    /* LEG_REAR_LEFT    (3) */ { 32, 31, 30 },
    /* LEG_MIDDLE_LEFT  (4) */ { 28, 27, 26 },
    /* LEG_FRONT_LEFT   (5) */ { 21, 22, 23 }
};

// Калибровочные смещения для каждой ноги
constexpr int LEG_OFFSETS[TOTAL_LEGS][NUM_JOINTS] = {
    /* FR */ {  0,   0,   0 },
    /* MR */ {  0,   0,   0 },
    /* RR */ {  0,   0,   0 },
    /* RL */ {  0,   0,   0 },
    /* ML */ {  0,   0,   0 },
    /* FL */ {  0,   0,   0 }
};

// Направления движения для походки
constexpr int LEG_FORWARD_DIRECTIONS[TOTAL_LEGS] = {
    /*FR*/ +1, /*MR*/ +1, /*RR*/ +1,
    /*RL*/ +1, /*ML*/ +1, /*FL*/ +1
};

// Направления сервоприводов для каждого сустава (зеркальное расположение!)
// Формат: [нога][сустав] где сустав: 0=COXA, 1=FEMUR, 2=TIBIA
constexpr int LEG_LIFT_DIRECTIONS[TOTAL_LEGS][3] = {
    /*FR*/ { +1, +1, -1 },  // Правые ноги: COXA +1, FEMUR +1, TIBIA -1
    /*MR*/ { +1, +1, -1 },
    /*RR*/ { +1, +1, -1 },
    /*RL*/ { -1, +1, -1 },  // Левые ноги: COXA -1, FEMUR +1, TIBIA -1 (зеркально!)
    /*ML*/ { -1, +1, -1 },
    /*FL*/ { -1, +1, -1 }
};

// Группы ног для tripod gait
constexpr LegID TRIPOD_GROUP_1[] = { LEG_FRONT_RIGHT, LEG_MIDDLE_LEFT, LEG_REAR_RIGHT };
constexpr LegID TRIPOD_GROUP_2[] = { LEG_FRONT_LEFT, LEG_MIDDLE_RIGHT, LEG_REAR_LEFT };

// ═══════════════════════════════════════════════════════════════
// GAIT TRAJECTORIES
// ═══════════════════════════════════════════════════════════════

// Траектория ПЕРЕНОСА: [COXA, FEMUR, TIBIA] на каждом шаге
constexpr int TRANSFER_TRAJ[4][3] = {
    {1320, 1700, 1250},  // Назад + начало подъёма
    {1500, 1800, 1150},  // Нейтраль + высоко
    {1680, 1800, 1150},  // Вперёд + высоко
    {1680, 1550, 1450}   // Вперёд + опускание
};

// Траектория ОПОРЫ: [COXA, FEMUR, TIBIA]
constexpr int SUPPORT_TRAJ[4][3] = {
    {1680, 1550, 1450},  // Вперёд + на земле
    {1600, 1520, 1480},  // Толчок назад
    {1450, 1490, 1510},  // Продолжение толчка
    {1320, 1470, 1530}   // Назад + финальный толчок
};

constexpr int TRAJ_STEPS = 4;

// ═══════════════════════════════════════════════════════════════
// TIMING CONFIGURATION
// ═══════════════════════════════════════════════════════════════

constexpr unsigned long STEP_DELAY = 150;       // Стандартный интервал (мс)
constexpr unsigned long FAST_STEP_DELAY = 120;  // Быстрая походка
constexpr unsigned long SLOW_STEP_DELAY = 200;  // Медленная походка

// ═══════════════════════════════════════════════════════════════
// SAFETY LIMITS
// ═══════════════════════════════════════════════════════════════

constexpr float MAX_COXA_ANGLE = 45.0f;   // градусы
constexpr float MAX_FEMUR_ANGLE = 90.0f;
constexpr float MAX_TIBIA_ANGLE = 90.0f;

constexpr int SAFE_MIN_PULSE = 1000;
constexpr int SAFE_MAX_PULSE = 2000;

// ═══════════════════════════════════════════════════════════════
// BATTERY MONITORING
// ═══════════════════════════════════════════════════════════════

constexpr int BATTERY_PIN = A0;
constexpr float VOLTAGE_DIVIDER = 4.2f;
constexpr float ADC_REF_VOLTAGE = 3.3f;
constexpr int ADC_RESOLUTION = 4095;
constexpr unsigned long BATTERY_UPDATE_INTERVAL = 5000; // мс

constexpr float BATTERY_MAX = 12.6f;  // Полная зарядка (3S LiPo)
constexpr float BATTERY_MIN = 9.0f;   // Разряжена
constexpr float BATTERY_LOW = 10.5f;  // Низкий заряд
constexpr float BATTERY_CRITICAL = 9.5f; // Критический

// ═══════════════════════════════════════════════════════════════
// NETWORK CONFIGURATION
// ═══════════════════════════════════════════════════════════════

constexpr int WEB_SERVER_PORT = 80;
constexpr int WEBSOCKET_PORT = 81;
constexpr unsigned long WIFI_TIMEOUT = 15000; // мс

// ═══════════════════════════════════════════════════════════════
// KINEMATICS PARAMETERS
// ═══════════════════════════════════════════════════════════════

constexpr float COXA_LENGTH = 50.0f;   // мм
constexpr float FEMUR_LENGTH = 80.0f;  // мм
constexpr float TIBIA_LENGTH = 120.0f; // мм

} // namespace Config
} // namespace Core

