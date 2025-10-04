#pragma once
#include "../../core/Types.h"
#include "../entities/Leg.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// KINEMATICS SERVICE INTERFACE
// Inverse/Forward Kinematics
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class IKinematicsService {
public:
    virtual ~IKinematicsService() = default;

    // Inverse Kinematics: Position -> Angles
    virtual bool calculateInverseKinematics(
        const Core::Position3D& targetPosition,
        Core::JointAngles& outAngles
    ) = 0;

    // Forward Kinematics: Angles -> Position
    virtual Core::Position3D calculateForwardKinematics(
        const Core::JointAngles& angles
    ) = 0;

    // Angles -> Servo Pulses (с учетом calibration)
    virtual Core::ServoPulses anglesToPulses(
        const Core::JointAngles& angles,
        std::shared_ptr<Leg> leg
    ) = 0;

    // Servo Pulses -> Angles
    virtual Core::JointAngles pulsesToAngles(
        const Core::ServoPulses& pulses,
        std::shared_ptr<Leg> leg
    ) = 0;

    // Проверка достижимости позиции
    virtual bool isPositionReachable(const Core::Position3D& position) = 0;
};

} // namespace Domain

