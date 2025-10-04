#pragma once
#include <memory>

// Infrastructure
#include "../infrastructure/hardware/ServoRepository.h"
#include "../infrastructure/hardware/BatteryMonitor.h"

// Domain
#include "../domain/entities/Body.h"
#include "../domain/services/SafetyService.h"
#include "../domain/services/KinematicsService.h"
#include "../domain/services/GaitService.h"

// Application
#include "../application/usecases/MoveForwardUseCase.h"
#include "../application/usecases/TurnUseCase.h"
#include "../application/usecases/PerformShakeUseCase.h"
#include "../application/usecases/PerformWaveUseCase.h"
#include "../application/usecases/AdjustBodyHeightUseCase.h"
#include "../application/usecases/AdjustBodyTiltUseCase.h"
#include "../application/usecases/AdjustBodyLeanUseCase.h"
#include "../application/usecases/AdjustBodyTwistUseCase.h"
#include "../application/usecases/PerformFullDiagnosticUseCase.h"
#include "../application/usecases/PerformJointTestUseCase.h"
#include "../application/usecases/PerformLegTestUseCase.h"
#include "../application/RobotController.h"

// ═══════════════════════════════════════════════════════════════
// DEPENDENCY INJECTION CONTAINER
// Простой DI Container для управления зависимостями
// ═══════════════════════════════════════════════════════════════

namespace DI {

class Container {
public:
    Container() : initialized_(false) {}

    // ═══════════════════════════════════════════════════════════
    // INITIALIZATION
    // ═══════════════════════════════════════════════════════════

    void initialize(HardwareSerial& serial) {
        if (initialized_) {
            Core::Logger::log(Core::Logger::WARNING, "Container already initialized");
            return;
        }

        Core::Logger::log(Core::Logger::INFO, "Initializing DI Container...");

        // ═══════════════════════════════════════════════════════════
        // INFRASTRUCTURE LAYER
        // ═══════════════════════════════════════════════════════════

        servos_ = std::make_shared<Infrastructure::ServoRepository>(serial);
        servos_->initialize();

        battery_ = std::make_shared<Infrastructure::BatteryMonitor>();
        battery_->enableSimulation(true);  // Можно изменить на false для реального ADC

        // ═══════════════════════════════════════════════════════════
        // DOMAIN LAYER
        // ═══════════════════════════════════════════════════════════

        body_ = std::make_shared<Domain::Body>();
        
        safety_ = std::make_shared<Domain::SafetyService>();
        
        kinematics_ = std::make_shared<Domain::KinematicsService>();
        
        gait_ = std::make_shared<Domain::GaitService>(
            body_,
            kinematics_,
            safety_,
            servos_
        );

        // ═══════════════════════════════════════════════════════════
        // APPLICATION LAYER
        // ═══════════════════════════════════════════════════════════

        moveForwardUseCase_ = std::make_shared<Application::MoveForwardUseCase>(
            gait_,
            safety_
        );

        turnUseCase_ = std::make_shared<Application::TurnUseCase>(
            gait_,
            safety_
        );

        shakeUseCase_ = std::make_shared<Application::PerformShakeUseCase>(
            body_,
            safety_,
            servos_
        );

        waveUseCase_ = std::make_shared<Application::PerformWaveUseCase>(
            body_,
            safety_,
            servos_
        );

        adjustHeightUseCase_ = std::make_shared<Application::AdjustBodyHeightUseCase>(
            body_,
            safety_,
            servos_
        );

        adjustTiltUseCase_ = std::make_shared<Application::AdjustBodyTiltUseCase>(
            body_,
            safety_,
            servos_
        );

        adjustLeanUseCase_ = std::make_shared<Application::AdjustBodyLeanUseCase>(
            body_,
            safety_,
            servos_
        );

        adjustTwistUseCase_ = std::make_shared<Application::AdjustBodyTwistUseCase>(
            body_,
            safety_,
            servos_
        );

        fullDiagnosticUseCase_ = std::make_shared<Application::PerformFullDiagnosticUseCase>(
            body_,
            safety_,
            servos_
        );

        jointTestUseCase_ = std::make_shared<Application::PerformJointTestUseCase>(
            body_,
            safety_,
            servos_
        );

        legTestUseCase_ = std::make_shared<Application::PerformLegTestUseCase>(
            body_,
            safety_,
            servos_
        );

        robotController_ = std::make_shared<Application::RobotController>(
            moveForwardUseCase_,
            turnUseCase_,
            shakeUseCase_,
            waveUseCase_,
            adjustHeightUseCase_,
            adjustTiltUseCase_,
            adjustLeanUseCase_,
            adjustTwistUseCase_,
            fullDiagnosticUseCase_,
            jointTestUseCase_,
            legTestUseCase_,
            gait_
        );

        initialized_ = true;
        Core::Logger::log(Core::Logger::INFO, "✅ DI Container initialized successfully");
    }

    // ═══════════════════════════════════════════════════════════
    // GETTERS
    // ═══════════════════════════════════════════════════════════

    std::shared_ptr<Application::RobotController> getRobotController() {
        return robotController_;
    }

    std::shared_ptr<Infrastructure::BatteryMonitor> getBatteryMonitor() {
        return battery_;
    }

    std::shared_ptr<Domain::ISafetyService> getSafetyService() {
        return safety_;
    }

    std::shared_ptr<Infrastructure::ServoRepository> getServoRepository() {
        return servos_;
    }

    bool isInitialized() const {
        return initialized_;
    }

private:
    bool initialized_;

    // Infrastructure
    std::shared_ptr<Infrastructure::ServoRepository> servos_;
    std::shared_ptr<Infrastructure::BatteryMonitor> battery_;

    // Domain
    std::shared_ptr<Domain::Body> body_;
    std::shared_ptr<Domain::SafetyService> safety_;
    std::shared_ptr<Domain::KinematicsService> kinematics_;
    std::shared_ptr<Domain::GaitService> gait_;

    // Application
    std::shared_ptr<Application::MoveForwardUseCase> moveForwardUseCase_;
    std::shared_ptr<Application::TurnUseCase> turnUseCase_;
    std::shared_ptr<Application::PerformShakeUseCase> shakeUseCase_;
    std::shared_ptr<Application::PerformWaveUseCase> waveUseCase_;
    std::shared_ptr<Application::AdjustBodyHeightUseCase> adjustHeightUseCase_;
    std::shared_ptr<Application::AdjustBodyTiltUseCase> adjustTiltUseCase_;
    std::shared_ptr<Application::AdjustBodyLeanUseCase> adjustLeanUseCase_;
    std::shared_ptr<Application::AdjustBodyTwistUseCase> adjustTwistUseCase_;
    std::shared_ptr<Application::PerformFullDiagnosticUseCase> fullDiagnosticUseCase_;
    std::shared_ptr<Application::PerformJointTestUseCase> jointTestUseCase_;
    std::shared_ptr<Application::PerformLegTestUseCase> legTestUseCase_;
    std::shared_ptr<Application::RobotController> robotController_;
};

} // namespace DI

