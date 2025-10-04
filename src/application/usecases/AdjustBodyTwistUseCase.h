#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// ADJUST BODY TWIST USE CASE
// Поворот тела вокруг вертикальной оси (twist left/right)
// ═══════════════════════════════════════════════════════════════

namespace Application {

class AdjustBodyTwistUseCase {
public:
    AdjustBodyTwistUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute(int offset) {
        // Validate
        if (abs(offset) > 100) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Twist offset too large: %d (max: ±100)", offset);
            return false;
        }

        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot adjust twist: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, 
            "📐 Adjusting body twist: %+d", offset);

        // Поворот: COXA всех ног работают вместе
        // ВАЖНО: учитываем LEG_LIFT_DIRECTIONS[leg][0] (COXA) для зеркального расположения!
        for (auto& leg : body_->getAllLegs()) {
            int legId = static_cast<int>(leg->getId());
            
            int coxa = Core::Config::NEUTRAL + leg->getCoxaOffset() + 
                      (offset * Core::Config::LEG_LIFT_DIRECTIONS[legId][0]); // COXA = index 0
            int femur = Core::Config::NEUTRAL + leg->getFemurOffset();
            int tibia = Core::Config::NEUTRAL + leg->getTibiaOffset();

            Core::ServoPulses pulses(
                safety_->constrainPulse(coxa),
                safety_->constrainPulse(femur),
                safety_->constrainPulse(tibia)
            );

            servos_->setLegPosition(leg->getId(), pulses, 300);
        }

        delay(300);
        Core::Logger::log(Core::Logger::INFO, "✅ Twist adjustment complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

