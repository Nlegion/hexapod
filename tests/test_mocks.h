#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cstdarg>

// Имитация Arduino типов и констант
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef int int8_t;
typedef int int16_t;
typedef long int32_t;

#define PROGMEM
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Имитация времени
class MockTimer {
private:
    static uint32_t current_time;
public:
    static uint32_t millis() { return current_time; }
    static void advance_time(uint32_t ms) { current_time += ms; }
    static void reset_time() { current_time = 0; }
};
uint32_t MockTimer::current_time = 0;

uint32_t millis() { return MockTimer::millis(); }
void delay(uint32_t ms) { MockTimer::advance_time(ms); }
void delayMicroseconds(uint32_t us) { /* no-op for tests */ }

// Имитация Serial интерфейса
class MockSerial {
private:
    std::vector<std::string> sent_commands;
    std::string buffer;
    
public:
    void begin(int baud) {
        std::cout << "[MOCK] Serial.begin(" << baud << ")" << std::endl;
    }
    
    void begin(int baud, int config, int rx, int tx) {
        std::cout << "[MOCK] Serial1.begin(" << baud << ", config, " << rx << ", " << tx << ")" << std::endl;
    }
    
    void print(const std::string& data) {
        sent_commands.push_back(data);
        std::cout << "[SERVO_CMD] " << data;
        if (data.back() != '\n') std::cout << std::endl;
    }
    
    void print(const char* str) {
        std::string data(str);
        sent_commands.push_back(data);
        std::cout << str;
    }
    
    void print(char* str) {
        std::string data(str);
        sent_commands.push_back(data);
        std::cout << str;
    }
    
    template<typename T>
    void print(T data) {
        print(std::to_string(data));
    }
    
    bool available() { return !buffer.empty(); }
    char read() {
        if (buffer.empty()) return 0;
        char c = buffer[0];
        buffer.erase(0, 1);
        return c;
    }
    
    // Тестовые методы
    const std::vector<std::string>& get_sent_commands() const { return sent_commands; }
    void clear_commands() { sent_commands.clear(); }
    void simulate_input(const std::string& data) { buffer += data; }
    
    int last_servo_channel() const {
        if (sent_commands.empty()) return -1;
        std::string cmd = sent_commands.back();
        size_t pos = cmd.find("#");
        if (pos == std::string::npos) return -1;
        
        size_t end = cmd.find("P", pos);
        if (end == std::string::npos) return -1;
        
        return std::stoi(cmd.substr(pos + 1, end - pos - 1));
    }
    
    int last_servo_pulse() const {
        if (sent_commands.empty()) return -1;
        std::string cmd = sent_commands.back();
        size_t pos = cmd.find("P");
        if (pos == std::string::npos) return -1;
        
        size_t end = cmd.find("T", pos);
        if (end == std::string::npos) return -1;
        
        return std::stoi(cmd.substr(pos + 1, end - pos - 1));
    }
};

// Глобальные Serial объекты
MockSerial Serial;
MockSerial Serial1;

// Имитация Logger
namespace Logger {
    enum Level { DEBUG, INFO, WARNING, ERROR };
    
    void log(Level level, const char* format, ...) {
        const char* level_names[] = {"DEBUG", "INFO", "WARN", "ERROR"};
        std::cout << "[" << level_names[level] << "] " << format << std::endl;
    }
}

// Трекер состояния сервоприводов
class ServoTracker {
private:
    static std::map<int, int> servo_positions;
    
public:
    static void set_position(int servo, int pulse) {
        servo_positions[servo] = pulse;
        std::cout << "[SERVO] Servo " << servo << " -> " << pulse << "μs" << std::endl;
    }
    
    static int get_position(int servo) {
        auto it = servo_positions.find(servo);
        return (it != servo_positions.end()) ? it->second : 1500; // default neutral
    }
    
    static void reset_all() {
        servo_positions.clear();
    }
    
    static std::map<int, int> get_all_positions() {
        return servo_positions;
    }
    
    static void print_status() {
        std::cout << "\n=== SERVO STATUS ===" << std::endl;
        for (auto& pair : servo_positions) {
            std::cout << "Servo " << pair.first << ": " << pair.second << "μs" << std::endl;
        }
        std::cout << "===================" << std::endl;
    }
};

std::map<int, int> ServoTracker::servo_positions;

// Мок для системы логирования
class MockLogger {
public:
    enum Level { DEBUG, INFO, WARNING, ERROR };
    
    static void log(Level level, const char* format, ...) {
        const char* level_str[] = {"DEBUG", "INFO", "WARNING", "ERROR"};
        char buffer[512];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        std::cout << "[" << level_str[level] << "] " << buffer << std::endl;
    }
};
