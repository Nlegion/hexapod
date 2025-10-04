#pragma once
#include "../../domain/services/ISafetyService.h"
#include "../../domain/repositories/IServoRepository.h"
#include "../../domain/entities/Body.h"
#include "../../core/Types.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// PERFORM JOINT TEST USE CASE
// Тест направлений суставов: каждый сустав двигается отдельно
// ═══════════════════════════════════════════════════════════════

namespace Application {

class PerformJointTestUseCase {
public:
    PerformJointTestUseCase(
        std::shared_ptr<Domain::Body> body,
        std::shared_ptr<Domain::ISafetyService> safety,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : body_(body), safety_(safety), servos_(servos) {}

    bool execute() {
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot perform joint test: Safety check failed");
            return false;
        }

        Core::Logger::log(Core::Logger::INFO, "🔍 Starting JOINT DIRECTIONS TEST");

        // Тестируем COXA (горизонтальное вращение)
        Core::Logger::log(Core::Logger::INFO, "Testing COXA joints...");
        for (auto& leg : body_->getAllLegs()) {
            int legId = static_cast<int>(leg->getId());
            const int coxaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][0];
            
            int coxa_neutral = Core::Config::NEUTRAL + leg->getCoxaOffset();
            int femur_neutral = Core::Config::NEUTRAL + leg->getFemurOffset();
            int tibia_neutral = Core::Config::NEUTRAL + leg->getTibiaOffset();

            // Двигаем COXA влево
            Core::ServoPulses left(
                safety_->constrainPulse(coxa_neutral + (150 * coxaDir)),
                femur_neutral,
                tibia_neutral
            );
            servos_->setLegPosition(leg->getId(), left, 300);
        }
        delay(500);

        // Возврат в нейтраль
        for (auto& leg : body_->getAllLegs()) {
            Core::ServoPulses neutral(
                Core::Config::NEUTRAL + leg->getCoxaOffset(),
                Core::Config::NEUTRAL + leg->getFemurOffset(),
                Core::Config::NEUTRAL + leg->getTibiaOffset()
            );
            servos_->setLegPosition(leg->getId(), neutral, 300);
        }
        delay(500);

        // Тестируем FEMUR + TIBIA (вертикальный подъём)
        Core::Logger::log(Core::Logger::INFO, "Testing FEMUR/TIBIA joints...");
        for (auto& leg : body_->getAllLegs()) {
            int legId = static_cast<int>(leg->getId());
            const int femurDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][1];
            const int tibiaDir = Core::Config::LEG_LIFT_DIRECTIONS[legId][2];
            
            int coxa_neutral = Core::Config::NEUTRAL + leg->getCoxaOffset();
            int femur_neutral = Core::Config::NEUTRAL + leg->getFemurOffset();
            int tibia_neutral = Core::Config::NEUTRAL + leg->getTibiaOffset();

            // Поднимаем
            Core::ServoPulses raised(
                coxa_neutral,
                safety_->constrainPulse(femur_neutral + (250 * femurDir)),
                safety_->constrainPulse(tibia_neutral + (250 * tibiaDir))
            );
            servos_->setLegPosition(leg->getId(), raised, 300);
        }
        delay(500);

        // Возврат в нейтраль
        for (auto& leg : body_->getAllLegs()) {
            Core::ServoPulses neutral(
                Core::Config::NEUTRAL + leg->getCoxaOffset(),
                Core::Config::NEUTRAL + leg->getFemurOffset(),
                Core::Config::NEUTRAL + leg->getTibiaOffset()
            );
            servos_->setLegPosition(leg->getId(), neutral, 300);
        }
        delay(300);

        Core::Logger::log(Core::Logger::INFO, "✅ JOINT DIRECTIONS TEST complete");
        return true;
    }

private:
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::ISafetyService> safety_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application

