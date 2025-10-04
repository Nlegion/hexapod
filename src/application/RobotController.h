#pragma once
#include "usecases/MoveForwardUseCase.h"
#include "usecases/TurnUseCase.h"
#include "usecases/PerformShakeUseCase.h"
#include "usecases/PerformWaveUseCase.h"
#include "usecases/AdjustBodyHeightUseCase.h"
#include "usecases/AdjustBodyTiltUseCase.h"
#include "usecases/AdjustBodyLeanUseCase.h"
#include "usecases/AdjustBodyTwistUseCase.h"
#include "usecases/PerformFullDiagnosticUseCase.h"
#include "usecases/PerformJointTestUseCase.h"
#include "usecases/PerformLegTestUseCase.h"
#include "../domain/services/IGaitService.h"
#include "../core/Logger.h"
#include <memory>
#include <string.h>

// ═══════════════════════════════════════════════════════════════
// ROBOT CONTROLLER
// Координатор всех Use Cases (Application Layer фасад)
// ═══════════════════════════════════════════════════════════════

namespace Application {

class RobotController {
public:
    RobotController(
        std::shared_ptr<MoveForwardUseCase> moveForward,
        std::shared_ptr<TurnUseCase> turn,
        std::shared_ptr<PerformShakeUseCase> shake,
        std::shared_ptr<PerformWaveUseCase> wave,
        std::shared_ptr<AdjustBodyHeightUseCase> adjustHeight,
        std::shared_ptr<AdjustBodyTiltUseCase> adjustTilt,
        std::shared_ptr<AdjustBodyLeanUseCase> adjustLean,
        std::shared_ptr<AdjustBodyTwistUseCase> adjustTwist,
        std::shared_ptr<PerformFullDiagnosticUseCase> fullDiagnostic,
        std::shared_ptr<PerformJointTestUseCase> jointTest,
        std::shared_ptr<PerformLegTestUseCase> legTest,
        std::shared_ptr<Domain::IGaitService> gait
    ) : moveForward_(moveForward),
        turn_(turn),
        shake_(shake),
        wave_(wave),
        adjustHeight_(adjustHeight),
        adjustTilt_(adjustTilt),
        adjustLean_(adjustLean),
        adjustTwist_(adjustTwist),
        fullDiagnostic_(fullDiagnostic),
        jointTest_(jointTest),
        legTest_(legTest),
        gait_(gait) {
        
        Core::Logger::log(Core::Logger::INFO, "RobotController initialized with all Use Cases");
    }

    // ═══════════════════════════════════════════════════════════
    // COMMAND HANDLING
    // ═══════════════════════════════════════════════════════════

    void handleCommand(const char* command) {
        Core::Logger::log(Core::Logger::INFO, "Command received: %s", command);

        // Movement commands
        if (strcmp(command, "FWD") == 0) {
            moveForward_->execute(1.0f);
        }
        else if (strcmp(command, "BWD") == 0) {
            gait_->startMovement(Core::MovementDirection::BACKWARD);
        }
        else if (strcmp(command, "LEFT") == 0) {
            turn_->executeTurnLeft(1.0f);
        }
        else if (strcmp(command, "RIGHT") == 0) {
            turn_->executeTurnRight(1.0f);
        }
        else if (strcmp(command, "STOP") == 0) {
            gait_->stopMovement();
        }
        // Reset command
        else if (strcmp(command, "RESET") == 0) {
            gait_->stopMovement();
            Core::Logger::log(Core::Logger::INFO, "Robot reset to neutral");
        }
        // Gesture commands
        else if (strcmp(command, "SHAKE") == 0) {
            gait_->stopMovement();
            shake_->execute();
        }
        else if (strcmp(command, "WAVE") == 0) {
            gait_->stopMovement();
            wave_->execute();
        }
        // Body adjustment commands (уменьшенные амплитуды для безопасности)
        else if (strcmp(command, "BODY_UP") == 0) {
            gait_->stopMovement();
            adjustLean_->execute(-50);  // Lean R (было adjustHeight)
        }
        else if (strcmp(command, "BODY_DOWN") == 0) {
            gait_->stopMovement();
            adjustLean_->execute(50);  // Lean L (было adjustHeight)
        }
        else if (strcmp(command, "HEAD_UP") == 0) {
            gait_->stopMovement();
            adjustTilt_->execute(50);  // Наклон вперёд
        }
        else if (strcmp(command, "HEAD_DOWN") == 0) {
            gait_->stopMovement();
            adjustTilt_->execute(-50);  // Наклон назад
        }
        else if (strcmp(command, "LEAN_LEFT") == 0) {
            gait_->stopMovement();
            adjustHeight_->execute(-60);  // Lower (было adjustLean)
        }
        else if (strcmp(command, "LEAN_RIGHT") == 0) {
            gait_->stopMovement();
            adjustHeight_->execute(60);  // Higher (было adjustLean)
        }
        else if (strcmp(command, "TWIST_LEFT") == 0) {
            gait_->stopMovement();
            adjustTwist_->execute(-40);  // Поворот влево (уменьшено с -60 до -40)
        }
        else if (strcmp(command, "TWIST_RIGHT") == 0) {
            gait_->stopMovement();
            adjustTwist_->execute(40);  // Поворот вправо (уменьшено с 60 до 40)
        }
        // Diagnostic commands
        else if (strcmp(command, "DIAGNOSTIC") == 0) {
            gait_->stopMovement();
            fullDiagnostic_->execute();
        }
        else if (strcmp(command, "JOINT_TEST") == 0) {
            gait_->stopMovement();
            jointTest_->execute();
        }
        else if (strcmp(command, "TRIPOD_TEST") == 0) {
            Core::Logger::log(Core::Logger::INFO, "🔍 Testing tripod gait");
            gait_->startMovement(Core::MovementDirection::FORWARD);
        }
        else if (strcmp(command, "EMERGENCY") == 0) {
            Core::Logger::log(Core::Logger::ERROR, "🚨 EMERGENCY STOP!");
            gait_->stopMovement();
        }
        // Individual leg tests
        else if (strncmp(command, "TEST_LEG_", 9) == 0) {
            int legId = atoi(command + 9);  // Извлекаем номер ноги
            if (legId >= 0 && legId < 6) {
                gait_->stopMovement();
                legTest_->execute(legId);
            }
        }
        // Unknown command
        else {
            Core::Logger::log(Core::Logger::WARNING, "Unknown command: %s", command);
        }
    }

    // ═══════════════════════════════════════════════════════════
    // UPDATE LOOP
    // ═══════════════════════════════════════════════════════════

    void update(unsigned long currentTime) {
        // Обновляем походку (неблокирующий цикл)
        gait_->updateGaitCycle(currentTime);
    }

    // ═══════════════════════════════════════════════════════════
    // STATE QUERIES
    // ═══════════════════════════════════════════════════════════

    bool isMoving() const {
        return gait_->isMoving();
    }

    const char* getStatusJSON() const {
        static char buffer[256];
        snprintf(buffer, sizeof(buffer),
            "{\"moving\":%s,\"direction\":%d,\"phase\":%d,\"step\":%d}",
            gait_->isMoving() ? "true" : "false",
            (int)gait_->getCurrentDirection(),
            (int)gait_->getCurrentPhase(),
            gait_->getCurrentStep()
        );
        return buffer;
    }

private:
    std::shared_ptr<MoveForwardUseCase> moveForward_;
    std::shared_ptr<TurnUseCase> turn_;
    std::shared_ptr<PerformShakeUseCase> shake_;
    std::shared_ptr<PerformWaveUseCase> wave_;
    std::shared_ptr<AdjustBodyHeightUseCase> adjustHeight_;
    std::shared_ptr<AdjustBodyTiltUseCase> adjustTilt_;
    std::shared_ptr<AdjustBodyLeanUseCase> adjustLean_;
    std::shared_ptr<AdjustBodyTwistUseCase> adjustTwist_;
    std::shared_ptr<PerformFullDiagnosticUseCase> fullDiagnostic_;
    std::shared_ptr<PerformJointTestUseCase> jointTest_;
    std::shared_ptr<PerformLegTestUseCase> legTest_;
    std::shared_ptr<Domain::IGaitService> gait_;
};

} // namespace Application

