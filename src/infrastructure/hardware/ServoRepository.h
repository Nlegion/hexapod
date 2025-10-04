#pragma once
#include "../../domain/repositories/IServoRepository.h"
#include "../../core/Config.h"
#include "../../core/Logger.h"
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
// SERVO REPOSITORY IMPLEMENTATION
// Взаимодействие с реальным servo controller (RTRobot)
// ═══════════════════════════════════════════════════════════════

namespace Infrastructure {

class ServoRepository : public Domain::IServoRepository {
public:
    explicit ServoRepository(HardwareSerial& serial) 
        : serial_(serial), isInitialized_(false) {
        
        // Инициализация кэша позиций
        for (int i = 0; i < 32; i++) {
            cachedPositions_[i] = Core::Config::NEUTRAL;
        }
    }

    // ═══════════════════════════════════════════════════════════
    // INITIALIZATION
    // ═══════════════════════════════════════════════════════════

    bool initialize() {
        // Отправляем команду инициализации контроллера
        serial_.write("#255P1500T1000\r\n");  // Команда сброса
        delay(100);
        
        // Очищаем буферы
        while (serial_.available()) {
            serial_.read();
        }
        
        isInitialized_ = true;
        Core::Logger::log(Core::Logger::INFO, "ServoRepository initialized");
        return true;
    }

    // ═══════════════════════════════════════════════════════════
    // SERVO CONTROL
    // ═══════════════════════════════════════════════════════════

    Core::CommandResult setServoPosition(
        uint8_t channel, 
        int pulse, 
        int time
    ) override {
        // Validation
        if (channel < 1 || channel > 32) {
            Core::Logger::log(Core::Logger::ERROR, 
                "Invalid servo channel: %d", channel);
            return Core::CommandResult::ERROR_INVALID_CHANNEL;
        }

        if (pulse < Core::Config::MIN_PULSE || pulse > Core::Config::MAX_PULSE) {
            Core::Logger::log(Core::Logger::WARNING, 
                "Pulse out of range: %d (constraining)", pulse);
            pulse = constrain(pulse, (int)Core::Config::MIN_PULSE, (int)Core::Config::MAX_PULSE);
        }

        // Формируем команду: #<channel>P<pulse>T<time>\r\n
        char command[32];
        snprintf(command, sizeof(command), "#%dP%dT%d\r\n", channel, pulse, time);
        
        // Отправляем
        serial_.write(command);
        
        // Обновляем кэш
        cachedPositions_[channel - 1] = pulse;

        return Core::CommandResult::SUCCESS;
    }

    int getServoPosition(uint8_t channel) const override {
        if (channel < 1 || channel > 32) {
            return Core::Config::NEUTRAL;
        }
        return cachedPositions_[channel - 1];
    }

    Core::CommandResult setLegPosition(
        Core::LegID leg,
        const Core::ServoPulses& pulses,
        int time
    ) override {
        uint8_t coxa_ch = Core::Config::LEG_SERVO_MAP[leg][Core::COXA];
        uint8_t femur_ch = Core::Config::LEG_SERVO_MAP[leg][Core::FEMUR];
        uint8_t tibia_ch = Core::Config::LEG_SERVO_MAP[leg][Core::TIBIA];

        // Отправляем все 3 команды
        setServoPosition(coxa_ch, pulses.coxa, time);
        setServoPosition(femur_ch, pulses.femur, time);
        setServoPosition(tibia_ch, pulses.tibia, time);

        return Core::CommandResult::SUCCESS;
    }

    void returnToNeutral() override {
        Core::Logger::log(Core::Logger::INFO, "Returning all servos to neutral");
        
        for (Core::LegID leg = Core::LEG_FRONT_RIGHT; leg < Core::TOTAL_LEGS; 
             leg = static_cast<Core::LegID>(leg + 1)) {
            
            Core::ServoPulses neutral(
                Core::Config::NEUTRAL + Core::Config::LEG_OFFSETS[leg][Core::COXA],
                Core::Config::NEUTRAL + Core::Config::LEG_OFFSETS[leg][Core::FEMUR],
                Core::Config::NEUTRAL + Core::Config::LEG_OFFSETS[leg][Core::TIBIA]
            );
            
            setLegPosition(leg, neutral, 500);
        }
    }

    bool isReady() const override {
        return isInitialized_;
    }

private:
    HardwareSerial& serial_;
    bool isInitialized_;
    int cachedPositions_[32];  // Кэш последних позиций

    // Используем Arduino macro constrain() напрямую, не создаем свою функцию
};

} // namespace Infrastructure

