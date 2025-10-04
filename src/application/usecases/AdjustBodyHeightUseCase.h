#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// ADJUST BODY HEIGHT USE CASE
// Регулировка высоты тела (поднять/опустить)
// ═══════════════════════════════════════════════════════════════

namespace Application {

class AdjustBodyHeightUseCase {
public:
    AdjustBodyHeightUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute(int offset) {
        // Validate input
        if (abs(offset) > 200) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Height offset too large: %d (max: ±200)", offset);
            return false;
        }

        // Safety check
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot adjust height: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, 
            "📐 Adjusting body height: %+d", offset);

        // Регулируем FEMUR и TIBIA всех ног одновременно
        // ВАЖНО: учитываем LEG_LIFT_DIRECTIONS для зеркального расположения!
        for (auto& leg : body_->getAllLegs()) {
            int legId = static_cast<int>(leg->getId());
            
            // Применяем offset с учетом направлений сервоприводов
            int femur = Core::Config::NEUTRAL + leg->getFemurOffset() + 
                        (offset * Core::Config::LEG_LIFT_DIRECTIONS[legId][1]); // FEMUR = index 1
            int tibia = Core::Config::NEUTRAL + leg->getTibiaOffset() - 
                        (offset * Core::Config::LEG_LIFT_DIRECTIONS[legId][2]); // TIBIA = index 2
            int coxa = Core::Config::NEUTRAL + leg->getCoxaOffset();

            Core::ServoPulses pulses(
                safety_->constrainPulse(coxa),
                safety_->constrainPulse(femur),
                safety_->constrainPulse(tibia)
            );

            servos_->setLegPosition(leg->getId(), pulses, 300);
        }

        delay(300);
        Core::Logger::log(Core::Logger::INFO, "✅ Height adjustment complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

