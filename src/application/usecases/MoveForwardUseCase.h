#pragma once
#include "domain/services/IGaitService.h"
#include "domain/services/ISafetyService.h"
#include "application/dto/MovementCommand.h"
#include "core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// MOVE FORWARD USE CASE
// ═══════════════════════════════════════════════════════════════

namespace Application {

class MoveForwardUseCase {
public:
    MoveForwardUseCase(
        std::shared_ptr<Domain::IGaitService> gait,
        std::shared_ptr<Domain::ISafetyService> safety
    ) : gait_(gait), safety_(safety) {}

    bool execute(float speed = 1.0f) {
        // Validate input
        if (speed <= 0.0f || speed > 1.0f) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Invalid speed: %.2f (must be 0.0-1.0)", speed);
            return false;
        }

        // Safety check
        if (!safety_->canMove()) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Cannot move forward: Safety check failed");
            return false;
        }

        // Execute movement
        gait_->setSpeed(speed);
        gait_->startMovement(Core::MovementDirection::FORWARD);
        
        Core::Logger::log(Core::Logger::INFO, 
            "Moving forward at speed %.2f", speed);
        return true;
    }

    void stop() {
        gait_->stopMovement();
        Core::Logger::log(Core::Logger::INFO, "Forward movement stopped");
    }

private:
    std::shared_ptr<Domain::IGaitService> gait_;
    std::shared_ptr<Domain::ISafetyService> safety_;
};

} // namespace Application

