#pragma once
#include "../../core/Types.h"

// ═══════════════════════════════════════════════════════════════
// SERVO REPOSITORY INTERFACE
// Абстракция для работы с сервоприводами (Dependency Inversion)
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class IServoRepository {
public:
    virtual ~IServoRepository() = default;

    // Установить позицию сервопривода
    virtual Core::CommandResult setServoPosition(
        uint8_t channel, 
        int pulse, 
        int time
    ) = 0;

    // Получить текущую позицию (если поддерживается)
    virtual int getServoPosition(uint8_t channel) const = 0;

    // Установить позицию всех суставов ноги
    virtual Core::CommandResult setLegPosition(
        Core::LegID leg,
        const Core::ServoPulses& pulses,
        int time
    ) = 0;

    // Вернуть все сервоприводы в нейтральное положение
    virtual void returnToNeutral() = 0;

    // Проверка готовности контроллера
    virtual bool isReady() const = 0;
};

} // namespace Domain

