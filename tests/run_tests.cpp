// Главный файл для запуска тестов гексапода
#include "test_framework.h"
#include "test_hexapod.cpp"
#include "test_domain_services.h"

int main() {
    std::cout << "🕷️ HEXAPOD TESTING SYSTEM" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "This system tests the hexapod logic before uploading to ESP32" << std::endl;
    std::cout << std::endl;
    
    // Запуск всех тестов
    TestFramework::run_all_tests();
    
    std::cout << std::endl;
    std::cout << "=== SERVO STATE AFTER TESTS ===" << std::endl;
    ServoTracker::print_status();
    
    return 0;
}
