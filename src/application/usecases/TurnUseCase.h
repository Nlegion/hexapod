#pragma once
#include "domain/services/IGaitService.h"
#include "domain/services/ISafetyService.h"
#include "core/Types.h"
#include "core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// TURN USE CASE (Left/Right)
// ═══════════════════════════════════════════════════════════════

namespace Application {

class TurnUseCase {
public:
    TurnUseCase(
        std::shared_ptr<Domain::IGaitService> gait,
        std::shared_ptr<Domain::ISafetyService> safety
    ) : gait_(gait), safety_(safety) {}

    bool executeTurnLeft(float speed = 1.0f) {
        return executeTurn(Core::MovementDirection::TURN_LEFT, speed, "left");
    }

    bool executeTurnRight(float speed = 1.0f) {
        return executeTurn(Core::MovementDirection::TURN_RIGHT, speed, "right");
    }

    void stop() {
        gait_->stopMovement();
        Core::Logger::log(Core::Logger::INFO, "Turn stopped");
    }

private:
    std::shared_ptr<Domain::IGaitService> gait_;
    std::shared_ptr<Domain::ISafetyService> safety_;

    bool executeTurn(Core::MovementDirection direction, float speed, const char* dirName) {
        // Validate
        if (speed <= 0.0f || speed > 1.0f) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Invalid speed: %.2f", speed);
            return false;
        }

        // Safety check
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot turn: Safety check failed");
            return false;
        }

        // Execute
        gait_->setSpeed(speed);
        gait_->startMovement(direction);
        
        Core::Logger::log(Core::Logger::INFO, 
            "Turning %s at speed %.2f", dirName, speed);
        return true;
    }
};

} // namespace Application

