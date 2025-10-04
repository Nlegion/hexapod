#pragma once
#include "IKinematicsService.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <math.h>

// ═══════════════════════════════════════════════════════════════
// KINEMATICS SERVICE IMPLEMENTATION
// Inverse/Forward Kinematics calculations
// ═══════════════════════════════════════════════════════════════

namespace Domain {

#ifndef PI
#define PI 3.14159265359f
#endif

#define RAD_TO_DEG(rad) ((rad) * 180.0f / PI)
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0f)
#define PULSE_TO_RAD(pulse) (DEG_TO_RAD((pulse - Core::Config::NEUTRAL) * 0.18f))
#define RAD_TO_PULSE(rad) (Core::Config::NEUTRAL + (int)(RAD_TO_DEG(rad) / 0.18f))

class KinematicsService : public IKinematicsService {
public:
    KinematicsService() {
        Core::Logger::log(Core::Logger::INFO, 
            "KinematicsService initialized: Coxa=%.1fmm, Femur=%.1fmm, Tibia=%.1fmm",
            Core::Config::COXA_LENGTH, 
            Core::Config::FEMUR_LENGTH, 
            Core::Config::TIBIA_LENGTH);
    }

    // ═══════════════════════════════════════════════════════════
    // INVERSE KINEMATICS: Position -> Angles
    // ═══════════════════════════════════════════════════════════

    bool calculateInverseKinematics(
        const Core::Position3D& targetPosition,
        Core::JointAngles& outAngles
    ) override {
        // Расчет горизонтального расстояния
        float horizontal_distance = sqrt(
            targetPosition.x * targetPosition.x + 
            targetPosition.y * targetPosition.y
        );
        
        float total_distance = sqrt(
            targetPosition.x * targetPosition.x +
            targetPosition.y * targetPosition.y +
            targetPosition.z * targetPosition.z
        );

        // Проверка досягаемости
        const float MAX_REACH = Core::Config::FEMUR_LENGTH + Core::Config::TIBIA_LENGTH;
        const float MIN_REACH = 20.0f;

        if (total_distance > MAX_REACH) {
            Core::Logger::log(Core::Logger::WARNING, 
                "IK: Target too far (%.1f > %.1f)", total_distance, MAX_REACH);
            return false;
        }

        if (total_distance < MIN_REACH) {
            Core::Logger::log(Core::Logger::WARNING, 
                "IK: Target too close (%.1f < %.1f)", total_distance, MIN_REACH);
            return false;
        }

        // 1. COXA ANGLE: поворот в горизонтальной плоскости
        outAngles.coxa = atan2(targetPosition.y, targetPosition.x);

        // 2. Расчет углов FEMUR и TIBIA
        float leg_reach = sqrt(
            (horizontal_distance - Core::Config::COXA_LENGTH) * 
            (horizontal_distance - Core::Config::COXA_LENGTH) + 
            targetPosition.z * targetPosition.z
        );

        if (leg_reach > (Core::Config::FEMUR_LENGTH + Core::Config::TIBIA_LENGTH) - 0.1f) {
            Core::Logger::log(Core::Logger::WARNING, "IK: Leg reach too far (%.1f)", leg_reach);
            return false;
        }

        // Теорема косинусов для треугольника femur-tibia-target
        float cos_tibia_angle = (
            Core::Config::FEMUR_LENGTH * Core::Config::FEMUR_LENGTH + 
            Core::Config::TIBIA_LENGTH * Core::Config::TIBIA_LENGTH - 
            leg_reach * leg_reach
        ) / (2 * Core::Config::FEMUR_LENGTH * Core::Config::TIBIA_LENGTH);

        // Проверка математической корректности
        if (cos_tibia_angle < -1.0f || cos_tibia_angle > 1.0f) {
            Core::Logger::log(Core::Logger::ERROR, 
                "IK: Math error - invalid cosine: %.3f", cos_tibia_angle);
            return false;
        }

        // 3. TIBIA ANGLE
        outAngles.tibia = PI - acos(cos_tibia_angle);

        // 4. FEMUR ANGLE
        float angle_to_target = atan2(targetPosition.z, horizontal_distance - Core::Config::COXA_LENGTH);
        float cos_alpha = (
            LEG_FEMUR_LENGTH * LEG_FEMUR_LENGTH + 
            leg_reach * leg_reach - 
            Core::Config::TIBIA_LENGTH * Core::Config::TIBIA_LENGTH
        ) / (2 * Core::Config::FEMUR_LENGTH * leg_reach);

        if (cos_alpha < -1.0f || cos_alpha > 1.0f) {
            Core::Logger::log(Core::Logger::ERROR, 
                "IK: Math error in femur calculation");
            return false;
        }

        float alpha = acos(cos_alpha);
        outAngles.femur = angle_to_target + alpha;

        // Проверка валидности углов
        if (!isAnglesValid(outAngles)) {
            Core::Logger::log(Core::Logger::WARNING, "IK: Invalid angles computed");
            return false;
        }

        return true;
    }

    // ═══════════════════════════════════════════════════════════
    // FORWARD KINEMATICS: Angles -> Position
    // ═══════════════════════════════════════════════════════════

    Core::Position3D calculateForwardKinematics(
        const Core::JointAngles& angles
    ) override {
        Core::Position3D result;

        // Позиция кончика коксы
        float coxa_x = Core::Config::COXA_LENGTH * cos(angles.coxa);
        float coxa_y = Core::Config::COXA_LENGTH * sin(angles.coxa);

        // Позиция кончика бедра
        float femur_horizontal = Core::Config::FEMUR_LENGTH * cos(angles.femur);
        float femur_vertical = Core::Config::FEMUR_LENGTH * sin(angles.femur);

        // Позиция кончика голени
        float tibia_horizontal = Core::Config::TIBIA_LENGTH * cos(angles.femur + angles.tibia - PI);
        float tibia_vertical = Core::Config::TIBIA_LENGTH * sin(angles.femur + angles.tibia - PI);

        // Финальная позиция
        float total_horizontal = coxa_x + femur_horizontal + tibia_horizontal;
        result.x = total_horizontal * cos(angles.coxa);
        result.y = total_horizontal * sin(angles.coxa);
        result.z = femur_vertical + tibia_vertical;

        return result;
    }

    // ═══════════════════════════════════════════════════════════
    // ANGLES <-> PULSES CONVERSION
    // ═══════════════════════════════════════════════════════════

    Core::ServoPulses anglesToPulses(
        const Core::JointAngles& angles,
        std::shared_ptr<Leg> leg
    ) override {
        Core::ServoPulses pulses;

        // Конвертация радианов в pulse с учетом calibration
        pulses.coxa = RAD_TO_PULSE(angles.coxa) + leg->getCoxaOffset();
        pulses.femur = RAD_TO_PULSE(angles.femur) + leg->getFemurOffset();
        pulses.tibia = RAD_TO_PULSE(angles.tibia) + leg->getTibiaOffset();

        return pulses;
    }

    Core::JointAngles pulsesToAngles(
        const Core::ServoPulses& pulses,
        std::shared_ptr<Leg> leg
    ) override {
        Core::JointAngles angles;

        // Конвертация pulse в радианы с учетом calibration
        angles.coxa = PULSE_TO_RAD(pulses.coxa - leg->getCoxaOffset());
        angles.femur = PULSE_TO_RAD(pulses.femur - leg->getFemurOffset());
        angles.tibia = PULSE_TO_RAD(pulses.tibia - leg->getTibiaOffset());

        return angles;
    }

    // ═══════════════════════════════════════════════════════════
    // VALIDATION
    // ═══════════════════════════════════════════════════════════

    bool isPositionReachable(const Core::Position3D& position) override {
        float distance = sqrt(
            position.x * position.x + 
            position.y * position.y + 
            position.z * position.z
        );

        const float MAX_REACH = Core::Config::FEMUR_LENGTH + Core::Config::TIBIA_LENGTH;
        const float MIN_REACH = 20.0f;

        return (distance >= MIN_REACH && distance <= MAX_REACH);
    }

private:
    static constexpr float LEG_FEMUR_LENGTH = Core::Config::FEMUR_LENGTH;

    bool isAnglesValid(const Core::JointAngles& angles) const {
        const float COXA_MIN = DEG_TO_RAD(-90.0f);
        const float COXA_MAX = DEG_TO_RAD(90.0f);
        const float FEMUR_MIN = DEG_TO_RAD(-135.0f);
        const float FEMUR_MAX = DEG_TO_RAD(135.0f);
        const float TIBIA_MIN = DEG_TO_RAD(-180.0f);
        const float TIBIA_MAX = DEG_TO_RAD(180.0f);

        return (angles.coxa >= COXA_MIN && angles.coxa <= COXA_MAX &&
                angles.femur >= FEMUR_MIN && angles.femur <= FEMUR_MAX &&
                angles.tibia >= TIBIA_MIN && angles.tibia <= TIBIA_MAX);
    }
};

} // namespace Domain

