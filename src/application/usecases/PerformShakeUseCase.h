#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// PERFORM SHAKE GESTURE USE CASE
// Выполнение жеста "пожатие лапы" (Front Right leg)
// ═══════════════════════════════════════════════════════════════

namespace Application {

class PerformShakeUseCase {
public:
    PerformShakeUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute() {
        // Safety check
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot perform shake: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, "🤝 Performing SHAKE gesture");

        // Используем переднюю правую ногу (LEG_FRONT_RIGHT)
        auto leg = body_->getLeg(Core::LEG_FRONT_RIGHT);

        // Учитываем LEG_LIFT_DIRECTIONS для правильного управления
        const int legId = 0; // FRONT_RIGHT
        const int coxaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][0];   // +1
        const int femurDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][1];  // +1
        const int tibiaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][2];  // -1

        // Базовые позиции
        int coxa_neutral = Core::Config::NEUTRAL + leg->getCoxaOffset();
        int femur_neutral = Core::Config::NEUTRAL + leg->getFemurOffset();
        int tibia_neutral = Core::Config::NEUTRAL + leg->getTibiaOffset();

        // === ШАГ 1: Вытянуть ногу вперёд и опустить вниз ===
        // COXA - максимально вперёд (к пользователю)
        // FEMUR - МАКСИМУМ ВНИЗ
        // TIBIA - МАКСИМУМ ВНИЗ
        Core::ServoPulses step1(
            safety_->constrainPulse(coxa_neutral + (450 * coxaDir)),   // МАКСИМАЛЬНО вперёд!
            safety_->constrainPulse(femur_neutral - (450 * femurDir)), // МАКСИМУМ ВНИЗ!
            safety_->constrainPulse(tibia_neutral - (450 * tibiaDir))  // МАКСИМУМ ВНИЗ!
        );
        servos_->setLegPosition(Core::LEG_FRONT_RIGHT, step1, 600);
        delay(600);

        // === ШАГ 2: "Пожатие" - ТОЛЬКО TIBIA дёргается вверх-вниз ===
        // COXA и FEMUR остаются на месте!
        for (int i = 0; i < 5; i++) {
            // TIBIA вверх (от максимума вниз)
            Core::ServoPulses up(
                step1.coxa,   // COXA остаётся вперёд
                step1.femur,  // FEMUR остаётся внизу
                safety_->constrainPulse(tibia_neutral - (200 * tibiaDir))  // TIBIA вверх
            );
            servos_->setLegPosition(Core::LEG_FRONT_RIGHT, up, 180);
            delay(180);

            // TIBIA вниз (обратно в максимум)
            servos_->setLegPosition(Core::LEG_FRONT_RIGHT, step1, 180);
            delay(180);
        }

        // === ШАГ 3: Вернуть в нейтраль ===
        Core::ServoPulses neutral(coxa_neutral, femur_neutral, tibia_neutral);
        servos_->setLegPosition(Core::LEG_FRONT_RIGHT, neutral, 600);
        delay(600);

        Core::Logger::log(Core::Logger::INFO, "✅ SHAKE gesture complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

