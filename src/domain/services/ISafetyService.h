#pragma once
#include "../../core/Types.h"
#include "../entities/Leg.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// SAFETY SERVICE INTERFACE
// Проверки безопасности движения
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class ISafetyService {
public:
    virtual ~ISafetyService() = default;

    // Проверка безопасности pulse значения
    virtual bool isPulseSafe(int pulse) const = 0;

    // Проверка безопасности угла
    virtual bool isAngleSafe(float angle, Core::JointID joint) const = 0;

    // Проверка безопасности позиции ноги
    virtual bool isLegPositionSafe(std::shared_ptr<Leg> leg) const = 0;

    // Может ли робот безопасно двигаться (батарея, состояние и т.д.)
    virtual bool canMove() const = 0;

    // Установить статус батареи (для проверки)
    virtual void setBatteryStatus(const Core::BatteryStatus& status) = 0;

    // Ограничить pulse значение безопасными пределами
    virtual int constrainPulse(int pulse) const = 0;
};

} // namespace Domain

