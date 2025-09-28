#pragma once

// Условные include для тестовой среды  
#ifdef TESTING_MODE
    // В тестах используем мок-версии
    #include "tests/test_config.h"
    #include "tests/test_safety.h" 
    #include "tests/test_mocks.h"
    #define Logger MockLogger
#else
    // В обычной Arduino среде  
    #include "config.h"
    #include "safety.h"
    #include "logger.h"
#endif

#include <math.h>

// Математические константы и макросы
#ifndef PI
#define PI 3.14159265359f
#endif
#define RAD_TO_DEG(rad) ((rad) * 180.0f / PI)
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0f)
#define PULSE_TO_RAD(pulse) (DEG_TO_RAD((pulse - NEUTRAL) * 0.18f))  // MG90: 1000μs=-90°, 2000μs=+90°
#define RAD_TO_PULSE(rad) (NEUTRAL + (int)(RAD_TO_DEG(rad) / 0.18f))

// Геометрические параметры ноги (используем значения из config.h)
// Физическая конфигурация: COXA (тазобедренный) -> FEMUR (бедро) -> TIBIA (голень)
constexpr float LEG_COXA_LENGTH = COXA_LENGTH;    // 39.0mm
constexpr float LEG_FEMUR_LENGTH = FEMUR_LENGTH;  // 43.0mm  
constexpr float LEG_TIBIA_LENGTH = TIBIA_LENGTH;  // 73.0mm

// Рабочие ограничения для каждого сустава (в радианах)
constexpr float COXA_MIN = DEG_TO_RAD(-90.0f);   // Расширенные ограничения поворота бедра
constexpr float COXA_MAX = DEG_TO_RAD(90.0f);
constexpr float FEMUR_MIN = DEG_TO_RAD(-135.0f); // Расширенные углы бедра для IK
constexpr float FEMUR_MAX = DEG_TO_RAD(135.0f);
constexpr float TIBIA_MIN = DEG_TO_RAD(-180.0f); // Полный диапазон сгибания голени
constexpr float TIBIA_MAX = DEG_TO_RAD(180.0f);

// Результат кинематических вычислений
enum class KinematicsResult {
    SUCCESS,
    TARGET_TOO_FAR,      // Целевая точка слишком далеко
    TARGET_TOO_CLOSE,    // Целевая точка слишком близко  
    INVALID_ANGLES,      // Недопустимые углы суставов
    MATH_ERROR          // Математическая ошибка (sqrt отрицательного числа и т.д.)
};

struct LegAngles {
    float coxa;   // Угол поворота тазобедренного сустава (радианы)
    float femur;  // Угол подъема бедренного сустава (радианы)  
    float tibia;  // Угол сгибания голенного сустава (радианы)
    
    // Проверка на валидность углов
    bool is_valid() const {
        return (coxa >= COXA_MIN && coxa <= COXA_MAX &&
                femur >= FEMUR_MIN && femur <= FEMUR_MAX &&
                tibia >= TIBIA_MIN && tibia <= TIBIA_MAX);
    }
};

struct LegPosition {
    float x, y, z;        // Координаты кончика ноги относительно центра коксы
    uint16_t duration;    // Время движения в миллисекундах
    LegAngles angles;     // Вычисленные углы суставов
    
    // Конструктор с координатами
    LegPosition(float x_pos = 0, float y_pos = 0, float z_pos = -100.0f, uint16_t dur = 1000) 
        : x(x_pos), y(y_pos), z(z_pos), duration(dur) {}
    
    // Расчет расстояния до центра коксы
    float distance_from_coxa() const {
        return sqrt(x*x + y*y + z*z);
    }
};

class LegController {
private:
    // Текущие позиции всех ног
    LegPosition current_positions[TOTAL_LEGS];
    
    // Максимальное расстояние досягаемости ноги
    constexpr static float MAX_REACH = LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH;
    constexpr static float MIN_REACH = 20.0f;  // Минимальное безопасное расстояние

public:
    // Инициализация контроллера  
    void init() {
        Logger::log(Logger::INFO, "LegController: Initializing with leg lengths: Coxa=%.1f, Femur=%.1f, Tibia=%.1f", 
                    LEG_COXA_LENGTH, LEG_FEMUR_LENGTH, LEG_TIBIA_LENGTH);
        
        // Устанавливаем все ноги в нейтральную позицию  
        for (int leg = 0; leg < TOTAL_LEGS; leg++) {
            current_positions[leg] = LegPosition(0, LEG_COXA_LENGTH, -80.0f);
            reset_pose(static_cast<LegID>(leg));
        }
    }
    
    // Сброс ноги в нейтральное положение
    void reset_pose(LegID leg_id) {
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int servo = LEG_SERVO_MAP[leg_id][joint];
            SafetySystem::set_servo(servo, NEUTRAL + LEG_OFFSETS[leg_id][joint]);
        }
    }

    // === ОБРАТНАЯ КИНЕМАТИКА (IK) ===
    // Вычисляет углы суставов для достижения целевой позиции кончика ноги
    KinematicsResult inverse_kinematics(const LegPosition& target, LegAngles& result) {
        // Расчет горизонтального расстояния от центра коксы
        float horizontal_distance = sqrt(target.x * target.x + target.y * target.y);
        float total_distance = target.distance_from_coxa();
        
        // Проверка досягаемости цели
        if (total_distance > MAX_REACH) {
            Logger::log(Logger::WARNING, "IK: Target too far (%.1f > %.1f)", total_distance, MAX_REACH);
            return KinematicsResult::TARGET_TOO_FAR;
        }
        
        if (total_distance < MIN_REACH) {
            Logger::log(Logger::WARNING, "IK: Target too close (%.1f < %.1f)", total_distance, MIN_REACH);
            return KinematicsResult::TARGET_TOO_CLOSE;
        }

        // 1. COXA ANGLE: поворот в горизонтальной плоскости
        result.coxa = atan2(target.y, target.x);

        // 2. Расчет углов FEMUR и TIBIA в вертикальной плоскости
        // После поворота коксы, работаем в плоскости, перпендикулярной оси коксы
        // Расстояние от центра коксы до цели в плоскости ноги
        float leg_reach = sqrt((horizontal_distance - LEG_COXA_LENGTH) * (horizontal_distance - LEG_COXA_LENGTH) + target.z * target.z);
        
        if (leg_reach > (LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH) - 0.1f) {
            Logger::log(Logger::WARNING, "IK: Leg reach too far (%.1f)", leg_reach);
            return KinematicsResult::TARGET_TOO_FAR;
        }

        // Используем теорему косинусов для треугольника femur-tibia-target
        float cos_tibia_angle = (LEG_FEMUR_LENGTH * LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH * LEG_TIBIA_LENGTH - leg_reach * leg_reach) 
                               / (2 * LEG_FEMUR_LENGTH * LEG_TIBIA_LENGTH);
        
        // Проверка на математическую корректность
        if (cos_tibia_angle < -1.0f || cos_tibia_angle > 1.0f) {
            Logger::log(Logger::ERROR, "IK: Math error - invalid cosine value: %.3f", cos_tibia_angle);
            return KinematicsResult::MATH_ERROR;
        }

        // 3. TIBIA ANGLE (корректировка для получения реального угла сгибания)
        float tibia_internal_angle = acos(cos_tibia_angle);  // Внутренний угол треугольника
        result.tibia = tibia_internal_angle - PI;  // Преобразование в угол сгибания
        
        // Если угол слишком экстремальный, используем альтернативное вычисление
        if (result.tibia < DEG_TO_RAD(-150.0f)) {
            result.tibia = PI - tibia_internal_angle; // Альтернативная геометрия
        }

        // 4. FEMUR ANGLE
        float alpha = atan2(-target.z, horizontal_distance - LEG_COXA_LENGTH);
        float beta = asin(LEG_TIBIA_LENGTH * sin(PI - result.tibia) / leg_reach);
        result.femur = alpha - beta;

        // Проверка ограничений суставов
        if (!result.is_valid()) {
            Logger::log(Logger::WARNING, "IK: Invalid joint angles - Coxa=%.1f° (limits ±60°), Femur=%.1f° (limits ±90°), Tibia=%.1f° (limits ±180°)",
                        RAD_TO_DEG(result.coxa), RAD_TO_DEG(result.femur), RAD_TO_DEG(result.tibia));
            Logger::log(Logger::DEBUG, "IK: leg_reach=%.1f, horizontal_distance=%.1f", leg_reach, horizontal_distance);
            return KinematicsResult::INVALID_ANGLES;
        }

        return KinematicsResult::SUCCESS;
    }

    // === ПРЯМАЯ КИНЕМАТИКА (FK) ===  
    // Вычисляет позицию кончика ноги по углам суставов
    LegPosition forward_kinematics(const LegAngles& angles) {
        LegPosition result;
        
        // Проверка валидности углов
        if (!angles.is_valid()) {
            Logger::log(Logger::WARNING, "FK: Invalid input angles");
            return result;  // Возвращает позицию (0,0,-100) по умолчанию
        }

        // Вычисление позиции кончика ноги
        // Используем стандартную кинематическую цепочку: Base -> Coxa -> Femur -> Tibia -> End Effector
        
        // 1. Позиция после поворота коксы (горизонтальная плоскость)
        float coxa_end_x = LEG_COXA_LENGTH * cos(angles.coxa);  
        float coxa_end_y = LEG_COXA_LENGTH * sin(angles.coxa);
        
        // 2. Позиция бедра в плоскости ноги  
        float femur_x = LEG_FEMUR_LENGTH * cos(angles.femur);
        float femur_z = -LEG_FEMUR_LENGTH * sin(angles.femur);  // Отрицательная Z (вниз)
        
        // 3. Позиция голени в плоскости ноги
        float tibia_x = LEG_TIBIA_LENGTH * cos(angles.femur + angles.tibia);
        float tibia_z = -LEG_TIBIA_LENGTH * sin(angles.femur + angles.tibia);
        
        // 4. Итоговая позиция в плоскости ноги
        float leg_x = femur_x + tibia_x;
        float leg_z = femur_z + tibia_z;
        
        // 5. Преобразование в глобальную систему координат
        result.x = coxa_end_x + leg_x * cos(angles.coxa);
        result.y = coxa_end_y + leg_x * sin(angles.coxa);
        result.z = leg_z;
        
        result.angles = angles;
        
        return result;
    }

    // === УПРАВЛЕНИЕ НОГОЙ ===
    // Устанавливает целевую позицию для конкретной ноги
    KinematicsResult set_target_position(LegID leg_id, const LegPosition& target) {
        if (leg_id >= TOTAL_LEGS) {
            Logger::log(Logger::ERROR, "Invalid leg ID: %d", leg_id);
            return KinematicsResult::INVALID_ANGLES;
        }

        LegAngles angles;
        KinematicsResult result = inverse_kinematics(target, angles);
        
        if (result != KinematicsResult::SUCCESS) {
            return result;
        }

        // Применение инверсии для левых ног (используем LEG_LIFT_DIRECTIONS)
        int coxa_pulse = RAD_TO_PULSE(angles.coxa * LEG_LIFT_DIRECTIONS[leg_id][COXA]) + LEG_OFFSETS[leg_id][COXA];
        int femur_pulse = RAD_TO_PULSE(angles.femur * LEG_LIFT_DIRECTIONS[leg_id][FEMUR]) + LEG_OFFSETS[leg_id][FEMUR];  
        int tibia_pulse = RAD_TO_PULSE(angles.tibia * LEG_LIFT_DIRECTIONS[leg_id][TIBIA]) + LEG_OFFSETS[leg_id][TIBIA];

        // Отправка команд сервоприводам через систему безопасности
        SafetySystem::set_servo(LEG_SERVO_MAP[leg_id][COXA], coxa_pulse);
        SafetySystem::set_servo(LEG_SERVO_MAP[leg_id][FEMUR], femur_pulse);
        SafetySystem::set_servo(LEG_SERVO_MAP[leg_id][TIBIA], tibia_pulse);

        // Обновление текущей позиции
        current_positions[leg_id] = target;
        current_positions[leg_id].angles = angles;

        Logger::log(Logger::DEBUG, "Leg %d: Position(%.1f,%.1f,%.1f) -> Angles(%.1f°,%.1f°,%.1f°) -> Pulses(%d,%d,%d)", 
                    leg_id, target.x, target.y, target.z,
                    RAD_TO_DEG(angles.coxa), RAD_TO_DEG(angles.femur), RAD_TO_DEG(angles.tibia),
                    coxa_pulse, femur_pulse, tibia_pulse);

        return KinematicsResult::SUCCESS;
    }

    // Получение текущей позиции ноги
    const LegPosition& get_current_position(LegID leg_id) const {
        return current_positions[leg_id];
    }

    // === ГРУППОВЫЕ ОПЕРАЦИИ ===
    // Обновление всех ног (пока заглушка - в будущем для интерполяции)  
    void update_all_legs() {
        // TODO: Реализовать интерполяцию между позициями
        // TODO: Синхронизация движений
    }

    // === ДИАГНОСТИЧЕСКИЕ ФУНКЦИИ ===
    void print_leg_status(LegID leg_id) {
        if (leg_id >= TOTAL_LEGS) return;
        
        const auto& pos = current_positions[leg_id];
        Logger::log(Logger::INFO, "Leg %d Status:", leg_id);
        Logger::log(Logger::INFO, "  Position: (%.1f, %.1f, %.1f)", pos.x, pos.y, pos.z);
        Logger::log(Logger::INFO, "  Angles: Coxa=%.1f° Femur=%.1f° Tibia=%.1f°", 
                    RAD_TO_DEG(pos.angles.coxa), RAD_TO_DEG(pos.angles.femur), RAD_TO_DEG(pos.angles.tibia));
        Logger::log(Logger::INFO, "  Distance from coxa: %.1f mm", pos.distance_from_coxa());
    }
};
