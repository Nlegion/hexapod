#pragma once
#include "usecases/MoveForwardUseCase.h"
#include "usecases/TurnUseCase.h"
#include "domain/services/IGaitService.h"
#include "core/Logger.h"
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
        std::shared_ptr<Domain::IGaitService> gait
    ) : moveForward_(moveForward),
        turn_(turn),
        gait_(gait) {
        
        Core::Logger::log(Core::Logger::INFO, "RobotController initialized");
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
    std::shared_ptr<Domain::IGaitService> gait_;
};

} // namespace Application

