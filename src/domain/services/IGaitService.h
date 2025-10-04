#pragma once
#include "core/Types.h"
#include "domain/entities/Body.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// GAIT SERVICE INTERFACE
// Tripod Gait Logic
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class IGaitService {
public:
    virtual ~IGaitService() = default;

    // ═══════════════════════════════════════════════════════════
    // GAIT CONTROL
    // ═══════════════════════════════════════════════════════════

    // Начать движение в заданном направлении
    virtual void startMovement(Core::MovementDirection direction) = 0;

    // Остановить движение
    virtual void stopMovement() = 0;

    // Обновить цикл походки (вызывается периодически)
    virtual void updateGaitCycle(unsigned long currentTime) = 0;

    // ═══════════════════════════════════════════════════════════
    // STATE QUERIES
    // ═══════════════════════════════════════════════════════════

    virtual bool isMoving() const = 0;
    virtual Core::MovementDirection getCurrentDirection() const = 0;
    virtual Core::GaitPhase getCurrentPhase() const = 0;
    virtual int getCurrentStep() const = 0;

    // ═══════════════════════════════════════════════════════════
    // SPEED CONTROL
    // ═══════════════════════════════════════════════════════════

    virtual void setSpeed(float speed) = 0;  // 0.0 - 1.0
    virtual float getSpeed() const = 0;
};

} // namespace Domain

