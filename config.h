// config.h
#pragma once

// Конфигурация сервоприводов
constexpr int LEG_SERVO_COUNT = 18;    // 6 ног × 3 сервопривода
constexpr int MIN_SERVO_PULSE = 500;   // Минимум для MG90S
constexpr int MAX_SERVO_PULSE = 2500;  // Максимум для MG90S
constexpr int DEFAULT_SERVO_POS = 1500;
constexpr int SERIAL1_TIMEOUT = 1000;

bool isMirroredLeg(int legIndex);

// Конфигурация контроллера CKX7651
constexpr int CONTROLLER_BAUDRATE = 9600;
constexpr int COMMAND_TIMEOUT = 3000;  // ms

// Конфигурация сети
constexpr int WIFI_TIMEOUT = 10000;  // ms

const int LEG_PINS[6][3] = {
  // Нога 1 (Слева)      2 (Центр слева)   3 (Слева задняя)
  { 1, 2, 3 },
  { 17, 18, 19 },
  { 29, 30, 31 },
  // Нога 4 (Справа)     5 (Центр справа)  6 (Справа задняя)
  { 5, 6, 7 },
  { 13, 14, 15 },
  { 25, 26, 27 }
};
