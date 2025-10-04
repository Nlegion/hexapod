#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// PERFORM LEG TEST USE CASE
// Тест отдельной ноги: поднять, повращать COXA, опустить
// ═══════════════════════════════════════════════════════════════

namespace Application {

class PerformLegTestUseCase {
public:
    PerformLegTestUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute(int legId) {
        if (legId < 0 || legId >= 6) {
            Core::Logger::log(Core::Logger::ERROR, "Invalid leg ID: %d", legId);
            return false;
        }

        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot test leg: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, "🦿 Testing leg %d", legId);

        auto leg = body_->getLeg(static_cast<Core::LegID>(legId));
        
        const int coxaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][0];
        const int femurDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][1];
        const int tibiaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][2];
        
        int coxa_neutral = Core::Config::NEUTRAL + leg->getCoxaOffset();
        int femur_neutral = Core::Config::NEUTRAL + leg->getFemurOffset();
        int tibia_neutral = Core::Config::NEUTRAL + leg->getTibiaOffset();

        // 1. Поднять ногу
        Core::ServoPulses raised(
            coxa_neutral,
            safety_->constrainPulse(femur_neutral + (350 * femurDir)),
            safety_->constrainPulse(tibia_neutral + (300 * tibiaDir))
        );
        servos_->setLegPosition(static_cast<Core::LegID>(legId), raised, 500);
        delay(500);

        // 2. Повращать COXA влево-вправо
        for (int i = 0; i < 3; i++) {
            // Влево
            Core::ServoPulses left(
                safety_->constrainPulse(coxa_neutral + (200 * coxaDir)),
                raised.femur,
                raised.tibia
            );
            servos_->setLegPosition(static_cast<Core::LegID>(legId), left, 300);
            delay(300);

            // Вправо
            Core::ServoPulses right(
                safety_->constrainPulse(coxa_neutral - (200 * coxaDir)),
                raised.femur,
                raised.tibia
            );
            servos_->setLegPosition(static_cast<Core::LegID>(legId), right, 300);
            delay(300);
        }

        // 3. Вернуть COXA в центр
        Core::ServoPulses center(
            coxa_neutral,
            raised.femur,
            raised.tibia
        );
        servos_->setLegPosition(static_cast<Core::LegID>(legId), center, 300);
        delay(300);

        // 4. Опустить ногу
        Core::ServoPulses neutral(coxa_neutral, femur_neutral, tibia_neutral);
        servos_->setLegPosition(static_cast<Core::LegID>(legId), neutral, 500);
        delay(500);

        Core::Logger::log(Core::Logger::INFO, "✅ Leg %d test complete", legId);
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

