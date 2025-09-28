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
  /*RL*/ {  -5, 130,   0 },  // Rear Left - увеличен FEMUR для компенсации зеркальности
  /*ML*/ {   0, 135,  -5 },  // Middle Left - увеличен FEMUR для компенсации зеркальности
  /*FL*/ {  10, 120, 10 }   // Front Left - увеличен FEMUR для компенсации зеркальности
};

// Направления движения для подъема ног
// ПОЛНАЯ ИНВЕРСИЯ ЛЕВЫХ НОГ: все суставы левой стороны физически зеркальны
constexpr int LEG_LIFT_DIRECTIONS[TOTAL_LEGS][NUM_JOINTS] = {
  /*FR*/ { +1, +1, -1 },  // Правые ноги: COXA +1, FEMUR +1, TIBIA -1
  /*MR*/ { +1, +1, -1 },
  /*RR*/ { +1, +1, -1 },
  /*RL*/ { -1, -1, +1 },  // Левые ноги: ВСЕ СУСТАВЫ ИНВЕРТИРОВАНЫ!
  /*ML*/ { -1, -1, +1 },  // COXA -1, FEMUR -1, TIBIA +1
  /*FL*/ { -1, -1, +1 }   // Полная зеркальная инверсия
};

// Servo пределы
constexpr int MIN_PULSE = 1000;
constexpr int MAX_PULSE = 2000;
constexpr int NEUTRAL = 1500;

// Траектории для трипоидной походки
constexpr int TRANSFER_TRAJ[4][3] = {
  {1480, 1600, 1350}, // Шаг 0: Начальный подъем + движение вперед
  {1520, 1650, 1300}, // Шаг 1: Максимальный подъем + максимальное движение вперед  
  {1520, 1550, 1400}, // Шаг 2: Начало опускания, но еще впереди
  {1500, 1500, 1500}  // Шаг 3: Нейтральная позиция
};

constexpr int SUPPORT_TRAJ[4][3] = {
  {1520, 1500, 1500}, // Шаг 0: Начальная позиция - впереди, на земле
  {1500, 1480, 1520}, // Шаг 1: Нейтральная позиция + небольшое напряжение
  {1480, 1450, 1550}, // Шаг 2: Движение назад + опора
  {1450, 1450, 1550}  // Шаг 3: Максимальное отталкивание назад
};
