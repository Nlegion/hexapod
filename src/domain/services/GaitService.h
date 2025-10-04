#pragma once
#include "IGaitService.h"
#include "IKinematicsService.h"
#include "ISafetyService.h"
#include "domain/repositories/IServoRepository.h"
#include "domain/entities/Body.h"
#include "core/Config.h"
#include "core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// GAIT SERVICE IMPLEMENTATION
// Tripod Gait Control Logic
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class GaitService : public IGaitService {
public:
    GaitService(
        std::shared_ptr<Body> body,
        std::shared_ptr<IKinematicsService> kinematics,
        std::shared_ptr<ISafetyService> safety,
        std::shared_ptr<IServoRepository> servos
    ) : body_(body),
        kinematics_(kinematics),
        safety_(safety),
        servos_(servos),
        isMoving_(false),
        currentDirection_(Core::MovementDirection::STOP),
        currentPhase_(Core::GaitPhase::PHASE1),
        currentStep_(0),
        speed_(1.0f),
        lastStepTime_(0) {
        
        Core::Logger::log(Core::Logger::INFO, "GaitService initialized");
    }

    // ═══════════════════════════════════════════════════════════
    // GAIT CONTROL
    // ═══════════════════════════════════════════════════════════

    void startMovement(Core::MovementDirection direction) override {
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, "Cannot start movement: Safety check failed");
            return;
        }

        if (direction == Core::MovementDirection::STOP) {
            stopMovement();
            return;
        }

        isMoving_ = true;
        currentDirection_ = direction;
        currentStep_ = 0;
        currentPhase_ = Core::GaitPhase::PHASE1;
        
        Core::Logger::log(Core::Logger::INFO, "Movement started: direction=%d", (int)direction);
    }

    void stopMovement() override {
        isMoving_ = false;
        currentDirection_ = Core::MovementDirection::STOP;
        Core::Logger::log(Core::Logger::INFO, "Movement stopped");
    }

    void updateGaitCycle(unsigned long currentTime) override {
        if (!isMoving_) return;

        // Проверка времени для следующего шага
        unsigned long stepDelay = getStepDelay();
        if (currentTime - lastStepTime_ < stepDelay) {
            return;
        }

        lastStepTime_ = currentTime;

        // Выполнить шаг походки
        executeGaitStep();

        // Переход к следующему шагу
        currentStep_++;
        if (currentStep_ >= Core::Config::TRAJ_STEPS) {
            currentStep_ = 0;
            // Переключение фазы
            currentPhase_ = (currentPhase_ == Core::GaitPhase::PHASE1) 
                ? Core::GaitPhase::PHASE2 
                : Core::GaitPhase::PHASE1;
        }
    }

    // ═══════════════════════════════════════════════════════════
    // STATE QUERIES
    // ═══════════════════════════════════════════════════════════

    bool isMoving() const override { return isMoving_; }
    
    Core::MovementDirection getCurrentDirection() const override { 
        return currentDirection_; 
    }
    
    Core::GaitPhase getCurrentPhase() const override { 
        return currentPhase_; 
    }
    
    int getCurrentStep() const override { 
        return currentStep_; 
    }

    // ═══════════════════════════════════════════════════════════
    // SPEED CONTROL
    // ═══════════════════════════════════════════════════════════

    void setSpeed(float speed) override {
        speed_ = constrain(speed, 0.0f, 1.0f);
        Core::Logger::log(Core::Logger::INFO, "Speed set to: %.2f", speed_);
    }

    float getSpeed() const override { return speed_; }

private:
    // Dependencies
    std::shared_ptr<Body> body_;
    std::shared_ptr<IKinematicsService> kinematics_;
    std::shared_ptr<ISafetyService> safety_;
    std::shared_ptr<IServoRepository> servos_;

    // State
    bool isMoving_;
    Core::MovementDirection currentDirection_;
    Core::GaitPhase currentPhase_;
    int currentStep_;
    float speed_;
    unsigned long lastStepTime_;

    // ═══════════════════════════════════════════════════════════
    // PRIVATE METHODS
    // ═══════════════════════════════════════════════════════════

    unsigned long getStepDelay() const {
        // Адаптивная скорость на основе speed_
        unsigned long baseDelay = Core::Config::STEP_DELAY;
        if (speed_ > 0.8f) {
            baseDelay = Core::Config::FAST_STEP_DELAY;
        } else if (speed_ < 0.5f) {
            baseDelay = Core::Config::SLOW_STEP_DELAY;
        }
        return baseDelay;
    }

    void executeGaitStep() {
        // Определяем какие ноги переносятся, какие опираются
        auto transferLegs = (currentPhase_ == Core::GaitPhase::PHASE1) 
            ? body_->getTripodGroup1() 
            : body_->getTripodGroup2();
        
        auto supportLegs = (currentPhase_ == Core::GaitPhase::PHASE1) 
            ? body_->getTripodGroup2() 
            : body_->getTripodGroup1();

        // Переносим ноги
        for (auto& leg : transferLegs) {
            applyTrajectory(leg, Core::Config::TRANSFER_TRAJ[currentStep_], true);
        }

        // Опорные ноги
        for (auto& leg : supportLegs) {
            applyTrajectory(leg, Core::Config::SUPPORT_TRAJ[currentStep_], false);
        }
    }

    void applyTrajectory(
        std::shared_ptr<Leg> leg, 
        const int trajectory[3],
        bool isTransfer
    ) {
        // Получаем базовые pulse значения из траектории
        int coxa_pulse = trajectory[0];
        int femur_pulse = trajectory[1];
        int tibia_pulse = trajectory[2];

        // Корректировка COXA на основе направления движения
        int coxa_direction = Core::Config::LEG_FORWARD_DIRECTIONS[leg->getId()];
        
        switch (currentDirection_) {
            case Core::MovementDirection::BACKWARD:
                coxa_direction *= -1;  // Инвертируем для движения назад
                break;
                
            case Core::MovementDirection::TURN_LEFT:
                if (leg->isLeftLeg()) {
                    coxa_direction *= -1;  // Левые ноги назад
                }
                break;
                
            case Core::MovementDirection::TURN_RIGHT:
                if (!leg->isLeftLeg()) {
                    coxa_direction *= -1;  // Правые ноги назад
                }
                break;
                
            case Core::MovementDirection::FORWARD:
            default:
                // Используем стандартное направление
                break;
        }

        // Применяем направление к COXA
        int coxa_offset = coxa_pulse - Core::Config::NEUTRAL;
        coxa_pulse = Core::Config::NEUTRAL + (coxa_offset * coxa_direction);

        // Применяем калибровочные смещения
        coxa_pulse += leg->getCoxaOffset();
        femur_pulse += leg->getFemurOffset();
        tibia_pulse += leg->getTibiaOffset();

        // Проверка безопасности
        coxa_pulse = safety_->constrainPulse(coxa_pulse);
        femur_pulse = safety_->constrainPulse(femur_pulse);
        tibia_pulse = safety_->constrainPulse(tibia_pulse);

        // Обновляем состояние ноги
        Core::ServoPulses pulses(coxa_pulse, femur_pulse, tibia_pulse);
        leg->setCurrentPulses(pulses);

        // Отправляем команды сервоприводам
        servos_->setLegPosition(leg->getId(), pulses, Core::Config::DEFAULT_TIME);
    }

    static float constrain(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};

} // namespace Domain

