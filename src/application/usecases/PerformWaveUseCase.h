#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// PERFORM WAVE GESTURE USE CASE
// Выполнение жеста "махание" (Front Left leg)
// ═══════════════════════════════════════════════════════════════

namespace Application {

class PerformWaveUseCase {
public:
    PerformWaveUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute() {
        // Safety check
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot perform wave: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, "👋 Performing WAVE gesture");

        // Используем левую переднюю ногу (LEG_FRONT_LEFT) - лучше видно
        auto leg = body_->getLeg(Core::LEG_FRONT_LEFT);

        // Учитываем LEG_LIFT_DIRECTIONS для правильного управления (ЗЕРКАЛЬНО!)
        const int legId = 5; // FRONT_LEFT
        const int coxaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][0];   // -1 (зеркально!)
        const int femurDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][1];  // +1
        const int tibiaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][2];  // -1

        // Базовые позиции
        int coxa_neutral = Core::Config::NEUTRAL + leg->getCoxaOffset();
        int femur_neutral = Core::Config::NEUTRAL + leg->getFemurOffset();
        int tibia_neutral = Core::Config::NEUTRAL + leg->getTibiaOffset();

        // === ШАГ 1: Поднять ногу МАКСИМАЛЬНО высоко ===
        Core::ServoPulses step1(
            coxa_neutral,
            safety_->constrainPulse(femur_neutral + (400 * femurDir)),  // Очень высоко!
            safety_->constrainPulse(tibia_neutral + (350 * tibiaDir))   // Подогнуть
        );
        servos_->setLegPosition(Core::LEG_FRONT_LEFT, step1, 500);
        delay(500);

        // === ШАГ 2: Махание COXA влево-вправо (увеличена амплитуда!) ===
        for (int i = 0; i < 5; i++) {
            // Наружу (с учетом зеркального направления)
            Core::ServoPulses out(
                safety_->constrainPulse(coxa_neutral + (250 * coxaDir)),  // Наружу
                step1.femur,
                step1.tibia
            );
            servos_->setLegPosition(Core::LEG_FRONT_LEFT, out, 200);
            delay(200);

            // Внутрь
            Core::ServoPulses in(
                safety_->constrainPulse(coxa_neutral - (200 * coxaDir)),  // Внутрь
                step1.femur,
                step1.tibia
            );
            servos_->setLegPosition(Core::LEG_FRONT_LEFT, in, 200);
            delay(200);
        }

        // === ШАГ 3: Вернуть COXA в центр, затем опустить ===
        Core::ServoPulses center(
            coxa_neutral,
            step1.femur,
            step1.tibia
        );
        servos_->setLegPosition(Core::LEG_FRONT_LEFT, center, 300);
        delay(300);

        // Опустить
        Core::ServoPulses neutral(coxa_neutral, femur_neutral, tibia_neutral);
        servos_->setLegPosition(Core::LEG_FRONT_LEFT, neutral, 600);
        delay(600);

        Core::Logger::log(Core::Logger::INFO, "✅ WAVE gesture complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

