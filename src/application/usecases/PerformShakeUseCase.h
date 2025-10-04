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

        // === ШАГ 1: Поднять ногу МАКСИМАЛЬНО высоко и вынести вперёд ===
        Core::ServoPulses step1(
            safety_->constrainPulse(coxa_neutral + (200 * coxaDir)),  // Вперёд (увеличено!)
            safety_->constrainPulse(femur_neutral + (350 * femurDir)), // Очень высоко!
            safety_->constrainPulse(tibia_neutral + (300 * tibiaDir))  // Выпрямить
        );
        servos_->setLegPosition(Core::LEG_FRONT_RIGHT, step1, 500);
        delay(500);

        // === ШАГ 2: "Пожатие" - быстрые движения вверх-вниз (увеличена амплитуда!) ===
        for (int i = 0; i < 5; i++) {
            // Вверх
            Core::ServoPulses up(
                step1.coxa,
                safety_->constrainPulse(femur_neutral + (400 * femurDir)),
                safety_->constrainPulse(tibia_neutral + (250 * tibiaDir))
            );
            servos_->setLegPosition(Core::LEG_FRONT_RIGHT, up, 150);
            delay(150);

            // Вниз
            Core::ServoPulses down(
                step1.coxa,
                safety_->constrainPulse(femur_neutral + (250 * femurDir)),
                safety_->constrainPulse(tibia_neutral + (400 * tibiaDir))
            );
            servos_->setLegPosition(Core::LEG_FRONT_RIGHT, down, 150);
            delay(150);
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

