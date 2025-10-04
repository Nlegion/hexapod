#pragma once
#include "core/Types.h"

// ═══════════════════════════════════════════════════════════════
// DATA TRANSFER OBJECTS
// Объекты для передачи данных между слоями
// ═══════════════════════════════════════════════════════════════

namespace Application {

struct MovementCommand {
    Core::MovementDirection direction;
    float speed;  // 0.0 - 1.0
    
    MovementCommand(Core::MovementDirection dir = Core::MovementDirection::STOP, float spd = 1.0f)
        : direction(dir), speed(spd) {}
};

struct ServoCommand {
    uint8_t channel;
    int pulse;
    int time;
    
    ServoCommand(uint8_t ch = 0, int p = 1500, int t = 200)
        : channel(ch), pulse(p), time(t) {}
};

struct GestureCommand {
    enum Type {
        SHAKE,
        WAVE,
        UNKNOWN
    };
    
    Type type;
    
    GestureCommand(Type t = UNKNOWN) : type(t) {}
};

struct BodyAdjustment {
    enum Type {
        HEIGHT_UP,
        HEIGHT_DOWN,
        TILT_FORWARD,
        TILT_BACKWARD,
        LEAN_LEFT,
        LEAN_RIGHT,
        TWIST_LEFT,
        TWIST_RIGHT
    };
    
    Type type;
    int amount;  // Величина изменения
    
    BodyAdjustment(Type t, int amt = 100) : type(t), amount(amt) {}
};

} // namespace Application

