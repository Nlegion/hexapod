#pragma once
#include <cstdint>

// ═══════════════════════════════════════════════════════════════
// CORE TYPES - Общие типы данных для всего проекта
// ═══════════════════════════════════════════════════════════════

namespace Core {

// Идентификаторы ног (топологический порядок)
enum LegID : uint8_t {
    LEG_FRONT_RIGHT = 0,
    LEG_MIDDLE_RIGHT = 1,
    LEG_REAR_RIGHT = 2,
    LEG_REAR_LEFT = 3,
    LEG_MIDDLE_LEFT = 4,
    LEG_FRONT_LEFT = 5,
    TOTAL_LEGS = 6
};

// Идентификаторы суставов
enum JointID : uint8_t {
    COXA = 0,   // Горизонтальное вращение
    FEMUR = 1,  // Вертикальное (бедро)
    TIBIA = 2,  // Вертикальное (голень)
    NUM_JOINTS = 3
};

// Фазы походки (tripod gait)
enum class GaitPhase : uint8_t {
    PHASE1,  // Группа 1 переносится, группа 2 опирается
    PHASE2   // Группа 2 переносится, группа 1 опирается
};

// Направление движения
enum class MovementDirection : uint8_t {
    STOP,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

// Состояние походки
enum class GaitState : uint8_t {
    IDLE,
    LIFT,
    MOVE,
    LOWER
};

// Результат выполнения команды
enum class CommandResult : uint8_t {
    SUCCESS,
    ERROR_TIMEOUT,
    ERROR_INVALID_CHANNEL,
    ERROR_INVALID_PULSE,
    ERROR_COMMUNICATION,
    ERROR_SAFETY_VIOLATION
};

// 3D позиция
struct Position3D {
    float x;
    float y;
    float z;
    
    Position3D() : x(0), y(0), z(0) {}
    Position3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// Углы суставов
struct JointAngles {
    float coxa;
    float femur;
    float tibia;
    
    JointAngles() : coxa(0), femur(0), tibia(0) {}
    JointAngles(float c, float f, float t) : coxa(c), femur(f), tibia(t) {}
};

// Pulse значения сервоприводов
struct ServoPulses {
    int coxa;
    int femur;
    int tibia;
    
    ServoPulses() : coxa(1500), femur(1500), tibia(1500) {}
    ServoPulses(int c, int f, int t) : coxa(c), femur(f), tibia(t) {}
};

// Статус батареи
struct BatteryStatus {
    float voltage;
    float percentage;
    bool isLow;
    bool isCritical;
    
    BatteryStatus() : voltage(0), percentage(0), isLow(false), isCritical(false) {}
};

} // namespace Core

