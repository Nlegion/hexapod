#pragma once
#include "Leg.h"
#include "core/Types.h"
#include <memory>
#include <array>

// ═══════════════════════════════════════════════════════════════
// BODY ENTITY - Сущность "Тело робота" (Domain Logic)
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class Body {
public:
    Body() {
        // Создаем 6 ног
        for (int i = 0; i < Core::TOTAL_LEGS; i++) {
            legs_[i] = std::make_shared<Leg>(static_cast<Core::LegID>(i));
        }
    }

    // ═══════════════════════════════════════════════════════════
    // GETTERS
    // ═══════════════════════════════════════════════════════════

    std::shared_ptr<Leg> getLeg(Core::LegID id) {
        return legs_[id];
    }

    const std::shared_ptr<Leg> getLeg(Core::LegID id) const {
        return legs_[id];
    }

    const std::array<std::shared_ptr<Leg>, Core::TOTAL_LEGS>& getAllLegs() const {
        return legs_;
    }

    // ═══════════════════════════════════════════════════════════
    // DOMAIN LOGIC
    // ═══════════════════════════════════════════════════════════

    // Проверка: все ли ноги в безопасном диапазоне
    bool allLegsInSafeRange() const {
        for (const auto& leg : legs_) {
            if (!leg->isInSafeRange()) {
                return false;
            }
        }
        return true;
    }

    // Получить ноги группы 1 (tripod group 1)
    std::array<std::shared_ptr<Leg>, 3> getTripodGroup1() {
        return {
            legs_[Core::LEG_FRONT_RIGHT],
            legs_[Core::LEG_MIDDLE_LEFT],
            legs_[Core::LEG_REAR_RIGHT]
        };
    }

    // Получить ноги группы 2 (tripod group 2)
    std::array<std::shared_ptr<Leg>, 3> getTripodGroup2() {
        return {
            legs_[Core::LEG_FRONT_LEFT],
            legs_[Core::LEG_MIDDLE_RIGHT],
            legs_[Core::LEG_REAR_LEFT]
        };
    }

    // Сброс всех ног в нейтральное положение
    void resetToNeutral() {
        Core::Position3D neutral(0, 0, 0);
        Core::JointAngles neutralAngles(0, 0, 0);
        Core::ServoPulses neutralPulses(
            Core::Config::NEUTRAL,
            Core::Config::NEUTRAL,
            Core::Config::NEUTRAL
        );

        for (auto& leg : legs_) {
            leg->setCurrentPosition(neutral);
            leg->setTargetPosition(neutral);
            leg->setCurrentAngles(neutralAngles);
            leg->setCurrentPulses(neutralPulses);
        }
    }

private:
    std::array<std::shared_ptr<Leg>, Core::TOTAL_LEGS> legs_;
};

} // namespace Domain

