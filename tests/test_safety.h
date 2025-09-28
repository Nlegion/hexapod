// Адаптированная версия SafetySystem для тестирования
#pragma once
#include "test_mocks.h"
#include "test_config.h"

class SafetySystem {
private:
    static bool initialized;
    static int speed;
    
public:
    static void init() {
        initialized = true;
        speed = 1000; // default speed
        std::cout << "[SAFETY] SafetySystem initialized" << std::endl;
    }
    
    static void set_servo(int servo, int pulse) {
        if (!initialized) {
            std::cout << "[SAFETY] ERROR: SafetySystem not initialized!" << std::endl;
            return;
        }
        
        // Валидация канала сервопривода
        if (servo < 1 || servo > 32) {
            std::cout << "[SAFETY] ERROR: Invalid servo channel " << servo << std::endl;
            return;
        }
        
        // Применение безопасных ограничений
        int safe_pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
        if (safe_pulse != pulse) {
            std::cout << "[SAFETY] WARNING: Pulse " << pulse << " clamped to " << safe_pulse << std::endl;
        }
        
        // Отправка команды через Serial1
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "#%dP%dT%d\r\n", servo, safe_pulse, speed);
        Serial1.print(buffer);
        
        // Отслеживание позиции сервопривода
        ServoTracker::set_position(servo, safe_pulse);
    }
    
    static void emergency_stop() {
        std::cout << "[SAFETY] EMERGENCY STOP ACTIVATED!" << std::endl;
        Serial1.print("#255P0T0\r\n");  // Стоп всех каналов
    }
    
    static int get_speed() {
        return speed;
    }
    
    static void set_speed(int new_speed) {
        speed = constrain(new_speed, 100, 2000);
    }
    
    static void update_load_monitor() {
        // В тестах ничего не делаем
    }
};

bool SafetySystem::initialized = false;
int SafetySystem::speed = 1000;
