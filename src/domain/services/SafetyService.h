#pragma once
#include "ISafetyService.h"
#include "core/Config.h"
#include "core/Logger.h"

// ═══════════════════════════════════════════════════════════════
// SAFETY SERVICE IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class SafetyService : public ISafetyService {
public:
    SafetyService() : batteryStatus_() {}

    bool isPulseSafe(int pulse) const override {
        return (pulse >= Core::Config::SAFE_MIN_PULSE && 
                pulse <= Core::Config::SAFE_MAX_PULSE);
    }

    bool isAngleSafe(float angle, Core::JointID joint) const override {
        switch (joint) {
            case Core::COXA:
                return abs(angle) <= Core::Config::MAX_COXA_ANGLE;
            case Core::FEMUR:
                return abs(angle) <= Core::Config::MAX_FEMUR_ANGLE;
            case Core::TIBIA:
                return abs(angle) <= Core::Config::MAX_TIBIA_ANGLE;
            default:
                return false;
        }
    }

    bool isLegPositionSafe(std::shared_ptr<Leg> leg) const override {
        if (!leg) return false;

        // Проверяем углы
        if (!leg->isInSafeRange()) {
            Core::Logger::log(Core::Logger::WARNING, 
                "Leg %d: Angles out of safe range", leg->getId());
            return false;
        }

        // Проверяем pulses
        if (!leg->arePulsesValid()) {
            Core::Logger::log(Core::Logger::WARNING, 
                "Leg %d: Pulses out of valid range", leg->getId());
            return false;
        }

        return true;
    }

    bool canMove() const override {
        // Проверка критического уровня батареи
        if (batteryStatus_.isCritical) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot move: Critical battery level (%.2fV)", 
                batteryStatus_.voltage);
            return false;
        }

        return true;
    }

    void setBatteryStatus(const Core::BatteryStatus& status) override {
        batteryStatus_ = status;
    }

    int constrainPulse(int pulse) const override {
        if (pulse < Core::Config::SAFE_MIN_PULSE) {
            return Core::Config::SAFE_MIN_PULSE;
        }
        if (pulse > Core::Config::SAFE_MAX_PULSE) {
            return Core::Config::SAFE_MAX_PULSE;
        }
        return pulse;
    }

private:
    Core::BatteryStatus batteryStatus_;
};

} // namespace Domain

