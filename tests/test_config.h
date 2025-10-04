// Адаптированная версия config.h для тестирования
#pragma once
#include "test_mocks.h"

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

// Конфигурация сервоприводов для каждой ноги [COXA, FEMUR, TIBIA]
constexpr uint8_t LEG_SERVO_MAP[TOTAL_LEGS][NUM_JOINTS] = {
  /* LEG_FRONT_RIGHT */ { 9, 10, 11 },
  /* LEG_MIDDLE_RIGHT */ { 5, 6, 7 },
  /* LEG_REAR_RIGHT */ { 1, 2, 3 },
  /* LEG_REAR_LEFT */ { 32, 31, 30 },
  /* LEG_MIDDLE_LEFT */ { 28, 27, 26 },
  /* LEG_FRONT_LEFT */ { 21, 22, 23 }
};

// Калибровочные смещения для каждой ноги [COXA, FEMUR, TIBIA]
constexpr int LEG_OFFSETS[TOTAL_LEGS][NUM_JOINTS] = {
  /*FR*/ { -10, 10, -10 },  // Front Right
  /*MR*/ {   0, -5,   5 },  // Middle Right
  /*RR*/ {   5,  0,   0 },  // Rear Right
  /*RL*/ {  -5,  0,   0 },  // Rear Left - базовые калибровочные смещения
  /*ML*/ {   0,  5,  -5 },  // Middle Left - базовые калибровочные смещения
  /*FL*/ {  10, -10, 10 }   // Front Left - базовые калибровочные смещения
};

// Направления движения для подъема ног
// ПОЛНАЯ ИНВЕРСИЯ ЛЕВЫХ НОГ: все суставы левой стороны физически зеркальны
constexpr int LEG_LIFT_DIRECTIONS[TOTAL_LEGS][NUM_JOINTS] = {
  /*FR*/ { +1, +1, -1 },  // Правые ноги: COXA +1, FEMUR +1, TIBIA -1
  /*MR*/ { +1, +1, -1 },
  /*RR*/ { +1, +1, -1 },
  /*RL*/ { -1, +1, -1 },  // Левые ноги: только COXA инвертирован, FEMUR и TIBIA как у правых!
  /*ML*/ { -1, +1, -1 },  // Зеркальные сервоприводы = те же команды для того же движения  
  /*FL*/ { -1, +1, -1 }   // Только направление поворота COXA отличается
};

// Направления для движения ВПЕРЁД/НАЗАД (только COXA!)
constexpr int LEG_FORWARD_DIRECTIONS[TOTAL_LEGS] = {
  /*FR*/ +1,  // Правые ноги: траектория применяется напрямую
  /*MR*/ +1,
  /*RR*/ +1,
  /*RL*/ +1,  // Левые ноги: для движения ВПЕРЁД траектория применяется БЕЗ инверсии!
  /*ML*/ +1,  // Зеркальные сервоприводы требуют те же команды для того же физического движения
  /*FL*/ +1   // Все ноги двигаются синхронно вперёд
};

// Servo пределы
constexpr int MIN_PULSE = 1000;
constexpr int MAX_PULSE = 2000;
constexpr int NEUTRAL = 1500;

// ИСПРАВЛЕННЫЕ траектории для движения ВПЕРЁД
// ОПТИМИЗИРОВАННЫЕ ТРАЕКТОРИИ для TRIPOD GAIT (версия 3.0)
// Амплитуда COXA увеличена с ±100 до ±180 для более заметного движения
constexpr int TRANSFER_TRAJ[4][3] = {
  {1320, 1700, 1250},  // COXA=-180 (НАЗАД!), FEMUR +200, TIBIA -250
  {1500, 1800, 1150},  // COXA=0 (нейтраль), FEMUR +300, TIBIA -350
  {1680, 1800, 1150},  // COXA=+180 (ВПЕРЁД!), FEMUR +300, TIBIA -350
  {1680, 1550, 1450}   // COXA=+180, FEMUR +50, TIBIA -50
};

constexpr int SUPPORT_TRAJ[4][3] = {
  {1680, 1550, 1450},  // COXA=+180 (ВПЕРЁД), FEMUR +50, TIBIA -50
  {1600, 1520, 1480},  // COXA=+100, FEMUR +20, TIBIA -20
  {1450, 1490, 1510},  // COXA=-50, FEMUR -10, TIBIA +10
  {1320, 1470, 1530}   // COXA=-180 (НАЗАД!), FEMUR -30, TIBIA +30
};

// Физические размеры сегментов ног для кинематики
constexpr float FEMUR_LENGTH = 43.0f;  // Бедро
constexpr float TIBIA_LENGTH = 73.0f;  // Голень
constexpr float COXA_LENGTH = 39.0f;   // Тазобедренная часть
