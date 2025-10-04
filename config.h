// === config.h === (исправленная версия)
#pragma once
#include <Arduino.h>

// Декларация типов
enum LegID {
  LEG_FRONT_RIGHT,   // 0
  LEG_MIDDLE_RIGHT,  // 1
  LEG_REAR_RIGHT,    // 2
  LEG_REAR_LEFT,     // 3
  LEG_MIDDLE_LEFT,   // 4
  LEG_FRONT_LEFT,    // 5
  TOTAL_LEGS         // Количество ног
};

enum class GaitPhase {
    PHASE1,  // Первая тройка ног в воздухе
    PHASE2   // Вторая тройка ног в воздухе
};

enum JointID {
  COXA,   // Сустав ближе к телу
  FEMUR,  // Бедренный сустав
  TIBIA,  // Голенный сустав
  NUM_JOINTS
};

constexpr float LEG_ORIENTATION[TOTAL_LEGS] = {
  15.0f,    // LEG_FRONT_RIGHT
  60.0f,    // LEG_MIDDLE_RIGHT
  120.0f,   // LEG_REAR_RIGHT
  -120.0f,  // LEG_REAR_LEFT
  -60.0f,   // LEG_MIDDLE_LEFT
  -15.0f    // LEG_FRONT_LEFT
};

// Конфигурация сервоприводов для каждой ноги [COXA, FEMUR, TIBIA]
constexpr uint8_t LEG_SERVO_MAP[TOTAL_LEGS][NUM_JOINTS] = {
  /* LEG_FRONT_RIGHT */ { 9, 10, 11 },  // COX, FEMUR, TIBIA
  /* LEG_MIDDLE_RIGHT */ { 5, 6, 7 },
  /* LEG_REAR_RIGHT */ { 1, 2, 3 },
  /* LEG_REAR_LEFT */ { 32, 31, 30 },    // Изменено с {32,31,30}
  /* LEG_MIDDLE_LEFT */ { 28, 27, 26 },  // Изменено с {28,27,26}
  /* LEG_FRONT_LEFT */ { 21, 22, 23 }    // Изменено с {21,22,23}
};

// Калибровочные смещения для каждой ноги [COXA, FEMUR, TIBIA]
// Основаны на анализе legacy кода и физических особенностей робота
constexpr int LEG_OFFSETS[TOTAL_LEGS][NUM_JOINTS] = {
  /*FR*/ { -10, 10, -10 },  // Front Right - инверсия по COXA и TIBIA
  /*MR*/ {   0, -5,   5 },  // Middle Right - небольшая коррекция
  /*RR*/ {   5,  0,   0 },  // Rear Right - коррекция по COXA
  /*RL*/ {  -5,  0,   0 },  // Rear Left - базовые калибровочные смещения
  /*ML*/ {   0,  5,  -5 },  // Middle Left - базовые калибровочные смещения 
  /*FL*/ {  10, -10, 10 }   // Front Left - базовые калибровочные смещения
};

// Направления движения для подъема ног (FEMUR/TIBIA)
// ПОЛНАЯ ИНВЕРСИЯ ЛЕВЫХ НОГ: все суставы левой стороны физически зеркальны
// [нога][сустав] = направление (+1 или -1, где 0 означает без движения)
constexpr int LEG_LIFT_DIRECTIONS[TOTAL_LEGS][NUM_JOINTS] = {
  /*FR*/ { +1, +1, -1 },  // Правые ноги: COXA +1, FEMUR +1, TIBIA -1
  /*MR*/ { +1, +1, -1 },  // Аналогично  
  /*RR*/ { +1, +1, -1 },  // Аналогично
  /*RL*/ { -1, +1, -1 },  // Левые ноги: только COXA инвертирован, FEMUR и TIBIA как у правых!
  /*ML*/ { -1, +1, -1 },  // Зеркальные сервоприводы = те же команды для того же движения
  /*FL*/ { -1, +1, -1 }   // Только направление поворота COXA отличается
};

// ВАЖНО: Направления для движения ВПЕРЁД/НАЗАД (только COXA!)
// Для движения вперёд ВСЕ ноги должны двигать COXA в ОДНОМ направлении относительно траектории
// Левые ноги НЕ инвертируют COXA для движения вперёд, т.к. они физически зеркальны
// Инверсия COXA из LEG_LIFT_DIRECTIONS используется только для поворотов (turn left/right)
constexpr int LEG_FORWARD_DIRECTIONS[TOTAL_LEGS] = {
  /*FR*/ +1,  // Правые ноги: траектория применяется напрямую
  /*MR*/ +1,
  /*RR*/ +1,
  /*RL*/ +1,  // Левые ноги: для движения ВПЕРЁД траектория применяется БЕЗ инверсии!
  /*ML*/ +1,  // Зеркальные сервоприводы требуют те же команды для того же физического движения
  /*FL*/ +1   // Все ноги двигаются синхронно вперёд
};

// Пределы углов безопасности
constexpr int ANGLE_LIMITS[NUM_JOINTS][2] = {
  { -45, 45 },  // Coxa
  { 20, 160 },  // Femur (расширенный диапазон)
  { 50, 130 }   // Tibia (суженный безопасный диапазон)
};

// Network
constexpr char SSID[] = "Homenet_plus";
constexpr char PASSWORD[] = "29pronto69";
constexpr int WIFI_TIMEOUT = 20;

// Servo - БЕЗОПАСНЫЕ пределы для MG90S на основе спецификаций
constexpr int SERVOS_PER_LEG = 3;
constexpr int MIN_PULSE = 1000;  // Безопасный минимум для MG90S (0°)
constexpr int MAX_PULSE = 2000;  // Безопасный максимум для MG90S (180°) 
constexpr int NEUTRAL = 1500;    // Нейтральная позиция (90°)

// Kinematics
constexpr float BODY_RADIUS = 65.0f;   // Оптимизирован радиус тела
constexpr float FEMUR_LENGTH = 43.0f;  // Уточнены длины сегментов
constexpr float TIBIA_LENGTH = 73.0f;
constexpr float COXA_LENGTH = 39.0f;  // Длина коксы
constexpr float MAX_STEP = 60.0f;
constexpr float STEP_DURATION = 0.8f;                      // Уменьшена длительность шага
constexpr float STEP_LENGTH = 30.0f;                       // Увеличен шаг
constexpr float STEP_HEIGHT = 40.0f;                       // Увеличена высота подъема
constexpr float MAX_ANGLES[3] = { 60.0f, 90.0f, 120.0f };  // Уточнены ограничения
constexpr float GAIT_SPEED = 0.8f;                         // Базовая скорость движения


// ═══════════════════════════════════════════════════════════════════════════════
// ОПТИМИЗИРОВАННЫЕ ТРАЕКТОРИИ для TRIPOD GAIT (версия 3.0)
// ═══════════════════════════════════════════════════════════════════════════════
// Формат: [шаг][COXA, FEMUR, TIBIA] - все значения в пределах MIN_PULSE(1000) - MAX_PULSE(2000)
//
// КРИТИЧЕСКИЕ ПРАВИЛА для прямолинейного движения:
// 1. СИММЕТРИЯ: TRANSFER и SUPPORT должны быть ЗЕРКАЛЬНО симметричны по COXA
// 2. АМПЛИТУДА: Достаточный размах COXA (±180) для заметного движения
// 3. ВЫСОТА: Достаточный подъём FEMUR/TIBIA для избежания волочения
// 4. СИНХРОННОСТЬ: Все ноги используют LEG_FORWARD_DIRECTIONS = +1
//
// TRANSFER (нога в воздухе): COXA от НАЗАД (-180) → ВПЕРЁД (+180)
// SUPPORT (нога на земле): COXA от ВПЕРЁД (+180) → НАЗАД (-180), толкая тело
//
constexpr int TRANSFER_TRAJ[4][3] = {
  // Фаза ПЕРЕНОСА: нога поднимается и выносится вперёд
  // Шаг 0: Старт из МАКСИМАЛЬНО задней позиции + начало подъёма
  {1320, 1700, 1250},  // COXA=-180 (НАЗАД!), FEMUR +200 (подъём), TIBIA -250 (подъём)
  
  // Шаг 1: Нейтраль + МАКСИМАЛЬНЫЙ подъём (нога высоко в воздухе)
  {1500, 1800, 1150},  // COXA=0 (нейтраль), FEMUR +300 (высоко!), TIBIA -350 (высоко!)
  
  // Шаг 2: МАКСИМАЛЬНО вперёд + остаёмся высоко
  {1680, 1800, 1150},  // COXA=+180 (ВПЕРЁД!), FEMUR +300, TIBIA -350
  
  // Шаг 3: Опускаемся в ПЕРЕДНЕЙ позиции (готовимся к опоре)
  {1680, 1550, 1450}   // COXA=+180 (вперёд), FEMUR +50, TIBIA -50 (на землю)
};

constexpr int SUPPORT_TRAJ[4][3] = {
  // Фаза ОПОРЫ: нога на земле толкает НАЗАД, двигая тело ВПЕРЁД
  // Шаг 0: Начало из МАКСИМАЛЬНО передней позиции (принимаем вес)
  {1680, 1550, 1450},  // COXA=+180 (ВПЕРЁД), FEMUR +50, TIBIA -50
  
  // Шаг 1: Мощный толчок назад (основная тяга)
  {1600, 1520, 1480},  // COXA=+100, FEMUR +20, TIBIA -20
  
  // Шаг 2: Продолжаем толкать назад (движение продолжается)
  {1450, 1490, 1510},  // COXA=-50, FEMUR -10, TIBIA +10
  
  // Шаг 3: МАКСИМАЛЬНО назад + финальный мощный толчок
  {1320, 1470, 1530}   // COXA=-180 (НАЗАД!), FEMUR -30, TIBIA +30
};

// ПРОВЕРКА СИММЕТРИИ:
// TRANSFER: COXA 1320 → 1500 → 1680 → 1680 (диапазон: -180 → +180)
// SUPPORT:  COXA 1680 → 1600 → 1450 → 1320 (диапазон: +180 → -180)
// ✅ Полная симметрия! Амплитуда: ±180 (вместо ±100)

// Safety
constexpr float MAX_SPEED = 50.0f;
constexpr float TORQUE_LIMIT = 2.0f;
constexpr float CURRENT_SAMPLE_TIME = 500;

// Battery monitoring
constexpr int BATTERY_PIN = A0;           // ADC пин для мониторинга батареи (можно изменить)
constexpr float VOLTAGE_DIVIDER = 4.2f;   // Коэффициент делителя напряжения (R1+R2)/R2
                                          // Например: R1=100kΩ, R2=33kΩ -> (100+33)/33 = 4.03
                                          // Настройте под ваш делитель напряжения!
constexpr float ADC_REF_VOLTAGE = 3.3f;   // Опорное напряжение ADC ESP32
constexpr int ADC_RESOLUTION = 4095;      // 12-bit ADC (0-4095)
constexpr unsigned long BATTERY_UPDATE_INTERVAL = 5000; // Обновление каждые 5 секунд

// Константы для трипоидной походки
constexpr uint16_t GAIT_DELAY = 150;        // Задержка между фазами
constexpr float INTERPOLATION_STEP = 0.6f;  // Шаг интерполяции
