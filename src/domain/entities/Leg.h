#pragma once
#include "core/Types.h"
#include "core/Config.h"

// ═══════════════════════════════════════════════════════════════
// LEG ENTITY - Сущность "Нога" (Domain Logic)
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class Leg {
public:
    explicit Leg(Core::LegID id) 
        : id_(id), 
          currentPosition_(0, 0, 0),
          targetPosition_(0, 0, 0),
          currentAngles_(0, 0, 0),
          currentPulses_(Core::Config::NEUTRAL, Core::Config::NEUTRAL, Core::Config::NEUTRAL) {}

    // ═══════════════════════════════════════════════════════════
    // GETTERS
    // ═══════════════════════════════════════════════════════════

    Core::LegID getId() const { return id_; }
    
    const Core::Position3D& getCurrentPosition() const { return currentPosition_; }
    const Core::Position3D& getTargetPosition() const { return targetPosition_; }
    
    const Core::JointAngles& getCurrentAngles() const { return currentAngles_; }
    const Core::ServoPulses& getCurrentPulses() const { return currentPulses_; }

    // ═══════════════════════════════════════════════════════════
    // SETTERS
    // ═══════════════════════════════════════════════════════════

    void setTargetPosition(const Core::Position3D& pos) {
        targetPosition_ = pos;
    }

    void setCurrentPosition(const Core::Position3D& pos) {
        currentPosition_ = pos;
    }

    void setCurrentAngles(const Core::JointAngles& angles) {
        currentAngles_ = angles;
    }

    void setCurrentPulses(const Core::ServoPulses& pulses) {
        currentPulses_ = pulses;
    }

    // ═══════════════════════════════════════════════════════════
    // DOMAIN LOGIC
    // ═══════════════════════════════════════════════════════════

    // Проверка, находятся ли углы в безопасном диапазоне
    bool isInSafeRange() const {
        return (abs(currentAngles_.coxa) <= Core::Config::MAX_COXA_ANGLE &&
                abs(currentAngles_.femur) <= Core::Config::MAX_FEMUR_ANGLE &&
                abs(currentAngles_.tibia) <= Core::Config::MAX_TIBIA_ANGLE);
    }

    // Проверка, находятся ли pulse значения в допустимом диапазоне
    bool arePulsesValid() const {
        return (currentPulses_.coxa >= Core::Config::SAFE_MIN_PULSE && 
                currentPulses_.coxa <= Core::Config::SAFE_MAX_PULSE &&
                currentPulses_.femur >= Core::Config::SAFE_MIN_PULSE && 
                currentPulses_.femur <= Core::Config::SAFE_MAX_PULSE &&
                currentPulses_.tibia >= Core::Config::SAFE_MIN_PULSE && 
                currentPulses_.tibia <= Core::Config::SAFE_MAX_PULSE);
    }

    // Получить servo каналы для этой ноги
    uint8_t getCoxaChannel() const {
        return Core::Config::LEG_SERVO_MAP[id_][Core::COXA];
    }

    uint8_t getFemurChannel() const {
        return Core::Config::LEG_SERVO_MAP[id_][Core::FEMUR];
    }

    uint8_t getTibiaChannel() const {
        return Core::Config::LEG_SERVO_MAP[id_][Core::TIBIA];
    }

    // Получить калибровочные смещения
    int getCoxaOffset() const {
        return Core::Config::LEG_OFFSETS[id_][Core::COXA];
    }

    int getFemurOffset() const {
        return Core::Config::LEG_OFFSETS[id_][Core::FEMUR];
    }

    int getTibiaOffset() const {
        return Core::Config::LEG_OFFSETS[id_][Core::TIBIA];
    }

    // Является ли нога левой
    bool isLeftLeg() const {
        return (id_ == Core::LEG_REAR_LEFT || 
                id_ == Core::LEG_MIDDLE_LEFT || 
                id_ == Core::LEG_FRONT_LEFT);
    }

private:
    Core::LegID id_;
    Core::Position3D currentPosition_;
    Core::Position3D targetPosition_;
    Core::JointAngles currentAngles_;
    Core::ServoPulses currentPulses_;
};

} // namespace Domain

