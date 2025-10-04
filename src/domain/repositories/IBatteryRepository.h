#pragma once
#include "../../core/Types.h"

// ═══════════════════════════════════════════════════════════════
// BATTERY REPOSITORY INTERFACE
// Абстракция для мониторинга батареи
// ═══════════════════════════════════════════════════════════════

namespace Domain {

class IBatteryRepository {
public:
    virtual ~IBatteryRepository() = default;

    // Прочитать текущее напряжение
    virtual float readVoltage() = 0;

    // Получить полный статус батареи
    virtual Core::BatteryStatus getStatus() = 0;

    // Обновить данные (если требуется)
    virtual void update() = 0;
};

} // namespace Domain

