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
  /*RL*/ {  -5,  0,   0 },  // Rear Left - инверсия COXA
  /*ML*/ {   0,  5,  -5 },  // Middle Left - инверсия по FEMUR/TIBIA
  /*FL*/ {  10, -10, 10 }   // Front Left - полная инверсия по отношению к FR
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

// Servo
constexpr int SERVOS_PER_LEG = 3;
constexpr int MIN_PULSE = 500;   // Минимальный импульс для MG90S
constexpr int MAX_PULSE = 2500;  // Максимальный импульс
constexpr int NEUTRAL = 1500;

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


// Фиксированные траектории для трипоидной походки
constexpr int TRANSFER_TRAJ[4][3] = {
  {1450, 1600, 1300}, // Перенос: подъем и движение вперед
  {1470, 1550, 1350},
  {1500, 1500, 1400}, // Нейтраль
  {1530, 1550, 1350}
};

constexpr int SUPPORT_TRAJ[4][3] = {
  {1550, 1600, 1300}, // Опора: толчок назад
  {1530, 1800, 1000},
  {1500, 1850, 950},  // Максимальный подъем
  {1470, 1800, 1000}
};

// Safety
constexpr float MAX_SPEED = 50.0f;
constexpr float TORQUE_LIMIT = 2.0f;
constexpr float CURRENT_SAMPLE_TIME = 500;

// Константы для трипоидной походки
constexpr uint16_t GAIT_DELAY = 150;        // Задержка между фазами
constexpr float INTERPOLATION_STEP = 0.6f;  // Шаг интерполяции
