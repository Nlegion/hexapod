#pragma once
#include "domain/repositories/IBatteryRepository.h"
#include "core/Config.h"
#include "core/Logger.h"
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
// BATTERY MONITOR IMPLEMENTATION
// Мониторинг батареи через ADC
// ═══════════════════════════════════════════════════════════════

namespace Infrastructure {

class BatteryMonitor : public Domain::IBatteryRepository {
public:
    BatteryMonitor() 
        : lastVoltage_(0.0f),
          simulatedVoltage_(11.8f),
          lastSimulationUpdate_(0),
          useSimulation_(true) {  // По умолчанию используем симуляцию
        
        pinMode(Core::Config::BATTERY_PIN, INPUT);
        Core::Logger::log(Core::Logger::INFO, "BatteryMonitor initialized (SIMULATION MODE)");
    }

    void enableSimulation(bool enable) {
        useSimulation_ = enable;
        if (enable) {
            Core::Logger::log(Core::Logger::INFO, "Battery simulation ENABLED");
        } else {
            Core::Logger::log(Core::Logger::INFO, "Battery simulation DISABLED - using real ADC");
        }
    }

    float readVoltage() override {
        if (useSimulation_) {
            return readSimulatedVoltage();
        } else {
            return readRealVoltage();
        }
    }

    Core::BatteryStatus getStatus() override {
        Core::BatteryStatus status;
        status.voltage = readVoltage();
        
        // Рассчитываем процент заряда
        status.percentage = calculatePercentage(status.voltage);
        
        // Проверяем уровни
        status.isLow = (status.voltage < Core::Config::BATTERY_LOW);
        status.isCritical = (status.voltage < Core::Config::BATTERY_CRITICAL);
        
        return status;
    }

    void update() override {
        // Периодическое обновление (если нужно)
        lastVoltage_ = readVoltage();
    }

private:
    float lastVoltage_;
    float simulatedVoltage_;
    unsigned long lastSimulationUpdate_;
    bool useSimulation_;

    // ═══════════════════════════════════════════════════════════
    // REAL ADC READING
    // ═══════════════════════════════════════════════════════════

    float readRealVoltage() {
        // Усредняем 10 измерений для точности
        int adc_sum = 0;
        for (int i = 0; i < 10; i++) {
            adc_sum += analogRead(Core::Config::BATTERY_PIN);
            delayMicroseconds(100);
        }
        int adc_value = adc_sum / 10;
        
        // Преобразуем ADC в напряжение
        float voltage = (float)adc_value / Core::Config::ADC_RESOLUTION * 
                       Core::Config::ADC_REF_VOLTAGE * 
                       Core::Config::VOLTAGE_DIVIDER;
        
        Core::Logger::log(Core::Logger::INFO, 
            "Battery ADC: raw=%d (%.1f%%), V_battery=%.2fV", 
            adc_value, 
            (float)adc_value / Core::Config::ADC_RESOLUTION * 100,
            voltage);
        
        return voltage;
    }

    // ═══════════════════════════════════════════════════════════
    // SIMULATED READING (для тестирования без hardware)
    // ═══════════════════════════════════════════════════════════

    float readSimulatedVoltage() {
        unsigned long now = millis();
        
        // Медленная "разрядка" (0.01V каждую минуту)
        if (now - lastSimulationUpdate_ > 60000) {
            simulatedVoltage_ -= 0.01f;
            lastSimulationUpdate_ = now;
            
            // Автоматическая "перезарядка" при критическом уровне
            if (simulatedVoltage_ < 9.0f) {
                simulatedVoltage_ = 12.0f;
                Core::Logger::log(Core::Logger::INFO, 
                    "🔌 Simulated battery recharged to 12.0V");
            }
        }
        
        return simulatedVoltage_;
    }

    float calculatePercentage(float voltage) const {
        float percentage = ((voltage - Core::Config::BATTERY_MIN) / 
                           (Core::Config::BATTERY_MAX - Core::Config::BATTERY_MIN)) * 100.0f;
        
        // Ограничиваем 0-100%
        if (percentage < 0) percentage = 0;
        if (percentage > 100) percentage = 100;
        
        return percentage;
    }
};

} // namespace Infrastructure

