#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// ADJUST BODY TILT USE CASE
// Наклон тела вперёд/назад (head up/down)
// ═══════════════════════════════════════════════════════════════

namespace Application {

class AdjustBodyTiltUseCase {
public:
    AdjustBodyTiltUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute(int offset) {
        // Validate
        if (abs(offset) > 150) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Tilt offset too large: %d (max: ±150)", offset);
            return false;
        }

        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot adjust tilt: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, 
            "📐 Adjusting body tilt: %+d", offset);

        // Передние ноги - в одну сторону, задние - в другую
        // ВАЖНО: учитываем LEG_LIFT_DIRECTIONS для зеркального расположения!
        for (auto& leg : body_->getAllLegs()) {
            Core::LegID id = leg->getId();
            int legId = static_cast<int>(id);
            
            // Определяем: передняя или задняя нога
            bool is_front = (id == Core::LEG_FRONT_RIGHT || id == Core::LEG_FRONT_LEFT);
            bool is_rear = (id == Core::LEG_REAR_RIGHT || id == Core::LEG_REAR_LEFT);
            
            int adjustment = 0;
            if (is_front) {
                adjustment = offset;  // Передние - в одну сторону
            } else if (is_rear) {
                adjustment = -offset; // Задние - в другую
            }
            // Средние не меняем (adjustment = 0)

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

            servos_->setLegPosition(id, pulses, 300);
        }

        delay(300);
        Core::Logger::log(Core::Logger::INFO, "✅ Tilt adjustment complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

