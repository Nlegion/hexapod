#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// PERFORM FULL DIAGNOSTIC USE CASE
// Полный тест всех ног: поднять и опустить каждую
// ═══════════════════════════════════════════════════════════════

namespace Application {

class PerformFullDiagnosticUseCase {
public:
    PerformFullDiagnosticUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute() {
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot perform diagnostic: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, "🔍 Starting FULL DIAGNOSTIC TEST");

        // Тестируем каждую ногу по очереди
        for (int legId = 0; legId < 6; legId++) {
            auto leg = body_->getLeg(static_cast<Core::LegID>(legId));
            
            Core::Logger::log(Core::Logger::INFO, "Testing leg %d", legId);
            
            int coxa_neutral = Core::Config::NEUTRAL + leg->getCoxaOffset();
            int femur_neutral = Core::Config::NEUTRAL + leg->getFemurOffset();
            int tibia_neutral = Core::Config::NEUTRAL + leg->getTibiaOffset();
            
            const int femurDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][1];
            const int tibiaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][2];

            // Поднять ногу
            Core::ServoPulses raised(
                coxa_neutral,
                safety_->constrainPulse(femur_neutral + (300 * femurDir)),
                safety_->constrainPulse(tibia_neutral + (300 * tibiaDir))
            );
            servos_->setLegPosition(static_cast<Core::LegID>(legId), raised, 400);
            delay(400);

            // Опустить ногу
            Core::ServoPulses neutral(coxa_neutral, femur_neutral, tibia_neutral);
            servos_->setLegPosition(static_cast<Core::LegID>(legId), neutral, 400);
            delay(400);
        }

        Core::Logger::log(Core::Logger::INFO, "✅ FULL DIAGNOSTIC TEST complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

