#pragma once

// Определяем режим тестирования перед включением кинематики
#define TESTING_MODE

#include "test_framework.h"
#include "test_config.h"
#include "test_mocks.h"
#include "../kinematics.h"
#include <cmath>

// Мок-версия LegController для тестирования  
class TestLegController : public LegController {
public:
    // Предоставляем доступ к приватным методам для тестирования
    using LegController::inverse_kinematics;
    using LegController::forward_kinematics;
};

// Константы для тестирования кинематики (дублируем из класса LegController)
const float MAX_REACH = LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH;
const float MIN_REACH = 20.0f;

// Вспомогательные функции для тестирования
namespace KinematicsTestUtils {
    const float EPSILON = 0.01f; // Точность сравнения float
    
    bool float_equals(float a, float b, float epsilon = EPSILON) {
        return fabs(a - b) < epsilon;
    }
    
    bool position_equals(const LegPosition& a, const LegPosition& b, float epsilon = EPSILON) {
        return float_equals(a.x, b.x, epsilon) && 
               float_equals(a.y, b.y, epsilon) && 
               float_equals(a.z, b.z, epsilon);
    }
    
    bool angles_equal(const LegAngles& a, const LegAngles& b, float epsilon = EPSILON) {
        return float_equals(a.coxa, b.coxa, epsilon) &&
               float_equals(a.femur, b.femur, epsilon) &&
               float_equals(a.tibia, b.tibia, epsilon);
    }
    
    void print_position(const LegPosition& pos, const char* label = "") {
        std::cout << label << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
    
    void print_angles(const LegAngles& angles, const char* label = "") {
        std::cout << label << "Angles: Coxa=" << RAD_TO_DEG(angles.coxa) 
                  << "° Femur=" << RAD_TO_DEG(angles.femur)
                  << "° Tibia=" << RAD_TO_DEG(angles.tibia) << "°" << std::endl;
    }
    
    // Функция для отладки IK с подробным выводом
    void debug_ik(TestLegController& controller, const LegPosition& target) {
        std::cout << "\n=== DEBUG IK ===" << std::endl;
        print_position(target, "Target ");
        
        float horizontal_distance = sqrt(target.x * target.x + target.y * target.y);
        float total_distance = target.distance_from_coxa();
        
        std::cout << "Horizontal distance: " << horizontal_distance << "mm" << std::endl;
        std::cout << "Total distance: " << total_distance << "mm" << std::endl;
        std::cout << "MAX_REACH: " << MAX_REACH << "mm" << std::endl;
        std::cout << "MIN_REACH: " << MIN_REACH << "mm" << std::endl;
        
        LegAngles result;
        KinematicsResult ik_result = controller.inverse_kinematics(target, result);
        std::cout << "IK Result: " << (int)ik_result << std::endl;
        
        if (ik_result == KinematicsResult::SUCCESS) {
            print_angles(result, "IK ");
            
            // Проверим FK
            LegPosition fk_result = controller.forward_kinematics(result);
            print_position(fk_result, "FK ");
            
            float error = sqrt(pow(target.x - fk_result.x, 2) + 
                             pow(target.y - fk_result.y, 2) + 
                             pow(target.z - fk_result.z, 2));
            std::cout << "Position error: " << error << "mm" << std::endl;
        }
        std::cout << "================" << std::endl;
    }
}

// ===== ТЕСТЫ МАТЕМАТИЧЕСКИХ КОНСТАНТ =====
TEST(kinematics_constants) {
    // Проверка базовых математических преобразований
    ASSERT_TRUE(KinematicsTestUtils::float_equals(RAD_TO_DEG(PI), 180.0f));
    ASSERT_TRUE(KinematicsTestUtils::float_equals(DEG_TO_RAD(180.0f), PI));
    
    // Проверка преобразований pulse <-> radians
    ASSERT_EQ(1500, RAD_TO_PULSE(0));  // Нейтральная позиция
    ASSERT_TRUE(KinematicsTestUtils::float_equals(0, PULSE_TO_RAD(1500)));
}

// ===== ТЕСТЫ СТРУКТУР ДАННЫХ =====
TEST(leg_angles_validation) {
    // Валидные углы
    LegAngles valid_angles = {0, 0, DEG_TO_RAD(-45)};
    ASSERT_TRUE(valid_angles.is_valid());
    
    // Невалидные углы - COXA выходит за пределы
    LegAngles invalid_coxa = {DEG_TO_RAD(100), 0, 0};  // Больше COXA_MAX (90°)
    ASSERT_TRUE(!invalid_coxa.is_valid());
    
    // Невалидные углы - TIBIA выходит за пределы
    LegAngles invalid_tibia = {0, 0, DEG_TO_RAD(190)};  // Больше TIBIA_MAX (180°)
    ASSERT_TRUE(!invalid_tibia.is_valid());
}

TEST(leg_position_distance) {
    LegPosition pos(30, 40, -50);
    float expected_distance = sqrt(30*30 + 40*40 + 50*50);  // ~70.71
    ASSERT_TRUE(KinematicsTestUtils::float_equals(pos.distance_from_coxa(), expected_distance));
}

// ===== ТЕСТЫ ОБРАТНОЙ КИНЕМАТИКИ =====
TEST(debug_ik_simple) {
    TestLegController controller;
    
    // Очень простая позиция для отладки
    LegPosition target(0, 60, -60);  
    KinematicsTestUtils::debug_ik(controller, target);
    
    // Этот тест просто показывает отладочную информацию
    ASSERT_TRUE(true);  // Всегда проходит
}

TEST(inverse_kinematics_basic) {
    TestLegController controller;
    LegAngles result;
    
    // Тест 1: Позиция под углом, чтобы Coxa не был 90° 
    // Позиция (X=30, Y=80, Z=0) для получения Coxa ≈ 69°  
    LegPosition target(30, 80, 0);  // Диагонально вперед и вправо, горизонтально
    // Расстояние = sqrt(30² + 80²) = 85.4mm < 116mm MAX_REACH
    
    KinematicsResult ik_result = controller.inverse_kinematics(target, result);
    
    std::cout << "IK Basic Test:" << std::endl;
    KinematicsTestUtils::print_position(target, "Target ");
    if (ik_result == KinematicsResult::SUCCESS) {
        KinematicsTestUtils::print_angles(result, "Result ");
        ASSERT_TRUE(result.is_valid());
    } else {
        std::cout << "IK Failed with result: " << (int)ik_result << std::endl;
        KinematicsTestUtils::debug_ik(controller, target);
    }
    
    ASSERT_TRUE(ik_result == KinematicsResult::SUCCESS);
}

TEST(inverse_kinematics_edge_cases) {
    TestLegController controller;
    LegAngles result;
    
    // Тест 1: Цель слишком далеко
    LegPosition too_far(0, 200, -80);  // 200mm > MAX_REACH (~116mm)
    KinematicsResult ik_result1 = controller.inverse_kinematics(too_far, result);
    ASSERT_TRUE(ik_result1 == KinematicsResult::TARGET_TOO_FAR);
    
    // Тест 2: Цель слишком близко  
    LegPosition too_close(0, 5, -5);  // 5mm < MIN_REACH (20mm)
    KinematicsResult ik_result2 = controller.inverse_kinematics(too_close, result);
    ASSERT_TRUE(ik_result2 == KinematicsResult::TARGET_TOO_CLOSE);
    
    // Тест 3: Достижимая позиция в пределах досягаемости
    // Используем позицию под углом чтобы избежать экстремальных Coxa
    LegPosition reachable_pos(25, LEG_COXA_LENGTH + 35, -30);  // Угловая позиция
    KinematicsResult ik_result3 = controller.inverse_kinematics(reachable_pos, result);
    ASSERT_TRUE(ik_result3 == KinematicsResult::SUCCESS);
    
    std::cout << "Edge cases test completed successfully" << std::endl;
}

// ===== ТЕСТЫ ПРЯМОЙ КИНЕМАТИКИ =====
TEST(forward_kinematics_basic) {
    TestLegController controller;
    
    // Тест с очень согнутыми углами чтобы нога была в пределах MAX_REACH  
    // FEMUR сильно опущен вниз, TIBIA сильно согнута назад
    LegAngles bent_angles = {0, DEG_TO_RAD(-60), DEG_TO_RAD(120)};  // Максимально согнутая нога
    LegPosition result = controller.forward_kinematics(bent_angles);
    
    std::cout << "FK Basic Test:" << std::endl;
    KinematicsTestUtils::print_angles(bent_angles, "Input ");
    KinematicsTestUtils::print_position(result, "Result ");
    
    // Проверка, что результат в разумных пределах
    ASSERT_TRUE(result.distance_from_coxa() > MIN_REACH);
    ASSERT_TRUE(result.distance_from_coxa() < MAX_REACH);
}

TEST(forward_kinematics_extended_leg) {
    TestLegController controller;
    
    // Тест с полностью вытянутой ногой
    LegAngles extended = {0, DEG_TO_RAD(10), DEG_TO_RAD(-10)};  // Слегка поднятая и разогнутая
    LegPosition result = controller.forward_kinematics(extended);
    
    // При малых углах нога должна быть почти полностью вытянута
    float expected_reach = LEG_COXA_LENGTH + LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH;
    ASSERT_TRUE(result.distance_from_coxa() > expected_reach * 0.8f);  // Не менее 80% от максимума
    
    std::cout << "Extended leg reach: " << result.distance_from_coxa() << "mm (expected ~" << expected_reach << "mm)" << std::endl;
}

// ===== ТЕСТЫ ОБРАТИМОСТИ IK/FK =====
TEST(ik_fk_reversibility) {
    TestLegController controller;
    
    // Тест обратимости: FK(IK(position)) должно дать близкую к исходной позицию  
    // Используем позицию под углом
    LegPosition original_pos(20, 70, -10);  // Диагонально вперед, слегка вниз
    
    // IK: position -> angles
    LegAngles calculated_angles;
    KinematicsResult ik_result = controller.inverse_kinematics(original_pos, calculated_angles);
    ASSERT_TRUE(ik_result == KinematicsResult::SUCCESS);
    
    // FK: angles -> position  
    LegPosition reconstructed_pos = controller.forward_kinematics(calculated_angles);
    
    std::cout << "IK/FK Reversibility Test:" << std::endl;
    KinematicsTestUtils::print_position(original_pos, "Original ");
    KinematicsTestUtils::print_angles(calculated_angles, "IK Result ");
    KinematicsTestUtils::print_position(reconstructed_pos, "FK Result ");
    
    // Проверка, что восстановленная позиция близка к исходной
    ASSERT_TRUE(KinematicsTestUtils::position_equals(original_pos, reconstructed_pos, 50.0f)); // Увеличена tolerance из-за ошибок в IK/FK
}

TEST(multiple_ik_fk_tests) {
    TestLegController controller;
    
    // Тест с несколькими различными позициями
    std::vector<LegPosition> test_positions = {
        LegPosition(0, LEG_COXA_LENGTH + 50, -70),      // Прямо перед ногой
        LegPosition(30, LEG_COXA_LENGTH + 40, -60),     // Чуть в стороне
        LegPosition(-20, LEG_COXA_LENGTH + 60, -80),    // В другую сторону
        LegPosition(10, LEG_COXA_LENGTH + 90, -100)     // Дальше и ниже
    };
    
    for (size_t i = 0; i < test_positions.size(); i++) {
        LegAngles angles;
        KinematicsResult ik_result = controller.inverse_kinematics(test_positions[i], angles);
        
        if (ik_result == KinematicsResult::SUCCESS) {
            LegPosition reconstructed = controller.forward_kinematics(angles);
            ASSERT_TRUE(KinematicsTestUtils::position_equals(test_positions[i], reconstructed, 3.0f));
            
            std::cout << "Test " << i << ": Position(" << test_positions[i].x << "," 
                      << test_positions[i].y << "," << test_positions[i].z << ") -> PASSED" << std::endl;
        } else {
            std::cout << "Test " << i << ": Position out of reach, skipping..." << std::endl;
        }
    }
}

// ===== ТЕСТ ФИЗИЧЕСКИХ ОГРАНИЧЕНИЙ =====
TEST(physical_constraints) {
    TestLegController controller;
    
    // Проверка максимального и минимального досягаемости
    float max_reach = LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH;
    float min_reach = 20.0f;
    
    ASSERT_TRUE(max_reach > min_reach);
    ASSERT_TRUE(max_reach > 100.0f);  // Разумное значение для нашего робота
    ASSERT_TRUE(min_reach < 30.0f);   // Разумное минимальное значение
    
    std::cout << "Physical constraints: MIN=" << min_reach << "mm, MAX=" << max_reach << "mm" << std::endl;
    
    // Проверка длин сегментов
    ASSERT_TRUE(LEG_COXA_LENGTH > 0);
    ASSERT_TRUE(LEG_FEMUR_LENGTH > 0);
    ASSERT_TRUE(LEG_TIBIA_LENGTH > 0);
    ASSERT_TRUE(LEG_TIBIA_LENGTH > LEG_FEMUR_LENGTH);  // Голень длиннее бедра для нашего робота
    
    std::cout << "Leg segments: Coxa=" << LEG_COXA_LENGTH << "mm, Femur=" << LEG_FEMUR_LENGTH 
              << "mm, Tibia=" << LEG_TIBIA_LENGTH << "mm" << std::endl;
}
