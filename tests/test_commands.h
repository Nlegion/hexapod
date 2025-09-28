// Адаптированная версия Commands для тестирования
#pragma once
#include "test_mocks.h"
#include "test_config.h"
#include "test_safety.h"
#include <cstdarg>
#include <cstdio>

// Результаты выполнения команд (для тестирования)
enum class CommandResult {
  SUCCESS,
  INVALID_SERVO,
  INVALID_PULSE,
  SERIAL_ERROR,
  TIMEOUT
};

// Статус контроллера (мок-версия для тестов)
class ControllerStatus {
private:
  static bool initialized;
  static unsigned long last_command_time;
  static int command_count;
  
public:
  static bool is_ready() { return initialized; }
  static void mark_command_sent() { 
    command_count++; 
  }
  static unsigned long time_since_last_command() { 
    return 0; // В тестах возвращаем 0
  }
  static int get_command_count() { return command_count; }
  static void set_initialized(bool state) { initialized = state; }
};

class Commands {
public:
    static CommandResult init_controller() {
        std::cout << "[COMMANDS] Initializing 32-channel servo controller" << std::endl;
        
        Serial1.print("#255P0T0\r\n");    // Стоп всех каналов
        Serial1.print("#0P1500T0\r\n");  // Сброс в нейтральное положение
        
        ControllerStatus::set_initialized(true);
        std::cout << "[COMMANDS] Controller initialized and buffers cleared" << std::endl;
        return CommandResult::SUCCESS;
    }

    static CommandResult send_servo_direct(int servo, int pulse) {
        // Валидация (упрощенная для тестов)
        if (servo < 1 || servo > 32) {
            std::cout << "[COMMANDS] ERROR: Invalid servo channel " << servo << std::endl;
            return CommandResult::INVALID_SERVO;
        }
        
        pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);

        char buffer[32];
        snprintf(buffer, sizeof(buffer), "#%dP%dT0\r\n", servo, pulse);
        Serial1.print(buffer);
        
        ServoTracker::set_position(servo, pulse);
        ControllerStatus::mark_command_sent();
        return CommandResult::SUCCESS;
    }
    
    static CommandResult reset_all_servos() {
        std::cout << "[COMMANDS] Resetting all servos to startup position" << std::endl;
        
        for (int leg = 0; leg < TOTAL_LEGS; leg++) {
            for (int joint = 0; joint < NUM_JOINTS; joint++) {
                int servo = LEG_SERVO_MAP[leg][joint];
                int startup_pulse = NEUTRAL + LEG_OFFSETS[leg][joint];
                startup_pulse = constrain(startup_pulse, MIN_PULSE, MAX_PULSE);
                CommandResult result = send_servo_direct(servo, startup_pulse);
                if (result != CommandResult::SUCCESS) {
                    std::cout << "[COMMANDS] ERROR: Failed to reset servo " << servo << std::endl;
                    return result;
                }
            }
        }
        
        std::cout << "[COMMANDS] Reset complete" << std::endl;
        return CommandResult::SUCCESS;
    }
    
    static void test_single_leg(int leg_id) {
        if (leg_id < 0 || leg_id >= TOTAL_LEGS) return;
        
        std::cout << "[COMMANDS] Testing leg " << leg_id << " SAFELY" << std::endl;
        const char* joint_names[] = {"COXA", "FEMUR", "TIBIA"};
        const int SAFE_RANGE = 200;
        
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int servo = LEG_SERVO_MAP[leg_id][joint];
            
            std::cout << "[COMMANDS] Testing leg " << leg_id << ", joint " << joint_names[joint] << " (servo " << servo << ")" << std::endl;
            
            // Движение в одну сторону
            int pulse1 = constrain(NEUTRAL + SAFE_RANGE, MIN_PULSE, MAX_PULSE);
            send_servo_direct(servo, pulse1);
            delay(100);
            
            // Движение в другую сторону
            int pulse2 = constrain(NEUTRAL - SAFE_RANGE, MIN_PULSE, MAX_PULSE);
            send_servo_direct(servo, pulse2);
            delay(100);
            
            // Возврат в нейтраль с калибровочным смещением
            int neutral_pulse = constrain(NEUTRAL + LEG_OFFSETS[leg_id][joint], MIN_PULSE, MAX_PULSE);
            send_servo_direct(servo, neutral_pulse);
            delay(50);
        }
        
        std::cout << "[COMMANDS] Leg " << leg_id << " test completed" << std::endl;
    }
};

// Определение статических переменных ControllerStatus для тестов
bool ControllerStatus::initialized = false;
unsigned long ControllerStatus::last_command_time = 0;
int ControllerStatus::command_count = 0;

