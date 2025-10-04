#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// ADJUST BODY LEAN USE CASE
// Наклон тела влево/вправо
// ═══════════════════════════════════════════════════════════════

namespace Application {

class AdjustBodyLeanUseCase {
public:
    AdjustBodyLeanUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute(int offset) {
        // Validate
        if (abs(offset) > 150) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Lean offset too large: %d (max: ±150)", offset);
            return false;
        }

        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot adjust lean: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, 
            "📐 Adjusting body lean: %+d", offset);

        // Левые ноги - в одну сторону, правые - в другую
        // ВАЖНО: учитываем LEG_LIFT_DIRECTIONS для зеркального расположения!
        for (auto& leg : body_->getAllLegs()) {
            int legId = static_cast<int>(leg->getId());
            bool is_left = leg->isLeftLeg();
            
            int adjustment = is_left ? offset : -offset;

            int femur = Core::Config::NEUTRAL + leg->getFemurOffset() + 
                        (adjustment * Core::Config::LEG_LIFT_DIRECTIONS[legId][1]);
            int tibia = Core::Config::NEUTRAL + leg->getTibiaOffset() - 
                        (adjustment * Core::Config::LEG_LIFT_DIRECTIONS[legId][2]);
            int coxa = Core::Config::NEUTRAL + leg->getCoxaOffset();

            Core::ServoPulses pulses(
                safety_->constrainPulse(coxa),
                safety_->constrainPulse(femur),
                safety_->constrainPulse(tibia)
            );

            servos_->setLegPosition(leg->getId(), pulses, 300);
        }

        delay(300);
        Core::Logger::log(Core::Logger::INFO, "✅ Lean adjustment complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

