#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "logger.h"
#include "page_html.h"
#include "safety.h"
#include "commands.h"
#include "kinematics.h"

WebServer server(80);
WebSocketsServer webSocket(81);
LegController hexapod;

// Добавляем переменные для управления шагами
int current_step = 0;
unsigned long last_step_time = 0;
const unsigned long STEP_DELAY = 200; // Интервал между шагами

bool is_moving = false;

enum class GaitState {
  IDLE,
  LIFT,
  MOVE,
  LOWER
};

GaitPhase current_phase = GaitPhase::PHASE1;
unsigned long phase_start_time = 0;
GaitState gait_state = GaitState::IDLE;
LegID active_leg = LEG_FRONT_RIGHT;
float progress = 0.0f;

void init_webserver();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
void handle_command(const char* cmd);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 4, 5);

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Logger::log(Logger::INFO, "Connecting to WiFi...");
  }
  Logger::log(Logger::INFO, "Connected. IP: %s", WiFi.localIP().toString().c_str());

  server.on("/", []() {
    server.send_P(200, "text/html", PAGE_HTML);
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  SafetySystem::init();
  hexapod.reset_pose(active_leg);
  Logger::log(Logger::INFO, "Ready. All servos in neutral position");
}

void loop() {
  webSocket.loop();
  server.handleClient();
  SafetySystem::update_load_monitor();

  handle_gait_cycle();
}

void handle_gait_cycle() {
  if (!is_moving) return;

  if (millis() - last_step_time >= STEP_DELAY) {
    current_step = (current_step + 1) % 4;
    last_step_time = millis();

    // Смена фазы после полного цикла
    if (current_step == 0) {
      current_phase = (current_phase == GaitPhase::PHASE1) ? GaitPhase::PHASE2 : GaitPhase::PHASE1;
      Logger::log(Logger::INFO, "Phase changed to %s",
          (current_phase == GaitPhase::PHASE1) ? "PHASE1" : "PHASE2");
    }

    // Обновляем позиции всех ног
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      bool is_transfer = (current_phase == GaitPhase::PHASE1 &&
          (leg == LEG_FRONT_RIGHT || leg == LEG_REAR_RIGHT || leg == LEG_MIDDLE_LEFT)) ||
          (current_phase == GaitPhase::PHASE2 &&
          (leg == LEG_MIDDLE_RIGHT || leg == LEG_REAR_LEFT || leg == LEG_FRONT_LEFT));

      const int (*traj)[3] = is_transfer ? TRANSFER_TRAJ : SUPPORT_TRAJ;

      // Применяем импульсы с учетом калибровочных смещений
      int coxa = traj[current_step][0] + LEG_OFFSETS[leg][COXA];
      int femur = traj[current_step][1] + LEG_OFFSETS[leg][FEMUR];
      int tibia = traj[current_step][2] + LEG_OFFSETS[leg][TIBIA];

      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], coxa);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], femur);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], tibia);
    }
  }
}

void handle_command(const char* cmd) {
  if (strcmp(cmd, "FWD") == 0) {
    is_moving = true;
    current_phase = GaitPhase::PHASE1;
    current_step = 0;
    last_step_time = millis();
    Logger::log(Logger::INFO, "Starting fixed trajectory gait");
  } else if (strcmp(cmd, "STOP") == 0) {
    Logger::log(Logger::INFO, "Executing STOP command");
    is_moving = false;
    // Возврат в нейтральное положение
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      hexapod.reset_pose(static_cast<LegID>(leg));
    }
  } else if (strcmp(cmd, "EMERGENCY") == 0) {
    Logger::log(Logger::ERROR, "EMERGENCY STOP ACTIVATED!");
    is_moving = false;
    SafetySystem::emergency_stop();
  } else if (strcmp(cmd, "CALIBRATE") == 0) {
    Commands::calibration_mode();
  } else if (strcmp(cmd, "DIAGNOSTIC") == 0) {
    Commands::diagnostic_sequence();
  } else if (strcmp(cmd, "RESET") == 0) {
    Commands::reset_all_servos();
  } else if (strncmp(cmd, "TEST_LEG_", 9) == 0) {
    int leg_id = atoi(cmd + 9); // Извлекаем номер ноги из "TEST_LEG_0"
    if (leg_id >= 0 && leg_id < TOTAL_LEGS) {
      Commands::test_single_leg(leg_id);
    } else {
      Logger::log(Logger::WARNING, "Invalid leg ID: %d", leg_id);
    }
  } else if (strcmp(cmd, "TRIPOD_TEST") == 0) {
    test_tripod_gait();
  } else if (strcmp(cmd, "JOINT_TEST") == 0) {
    test_joint_directions();
  } else if (strcmp(cmd, "TEST_SERVO32") == 0) {
    test_servo_32();
  } else {
    Logger::log(Logger::WARNING, "Unknown command: %s", cmd);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Logger::log(Logger::INFO, "Client %d connected", num);
      break;

    case WStype_TEXT:
      {
        char cmd[length+1];
        memcpy(cmd, payload, length);
        cmd[length] = '\0';
        handle_command(cmd);
        break;
      }

    case WStype_DISCONNECTED:
      Logger::log(Logger::INFO, "Client %d disconnected", num);
      break;
  }
}

void test_servo_32() {
  Logger::log(Logger::INFO, "=== СПЕЦИАЛЬНАЯ ДИАГНОСТИКА СЕРВОПРИВОДА 32 (RL COXA) ===");
  is_moving = false;
  
  int servo = 32;
  int neutral = NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA]; // -5
  
  Logger::log(Logger::INFO, "Servo 32 info: neutral_base=%d, offset=%d, final_neutral=%d", 
             NEUTRAL, LEG_OFFSETS[LEG_REAR_LEFT][COXA], neutral);
  
  // Очистка и сброс
  Logger::log(Logger::INFO, "Step 1: Emergency reset of servo 32");
  Commands::send_servo_direct(32, NEUTRAL);
  delay(2000);
  
  // Прямая команда нейтраль
  Logger::log(Logger::INFO, "Step 2: Setting servo 32 to calculated neutral %d", neutral);
  Commands::send_servo_direct(32, neutral);
  delay(2000);
  
  // Тест движения в разные стороны с разными методами
  int test_positions[] = {neutral - 150, neutral - 100, neutral - 50, neutral, 
                          neutral + 50, neutral + 100, neutral + 150};
  
  for (int i = 0; i < 7; i++) {
    int pos = constrain(test_positions[i], MIN_PULSE, MAX_PULSE);
    
    Logger::log(Logger::INFO, "Step %d: Testing position %d (offset %+d from neutral)", 
               i+3, pos, pos - neutral);
    
    // Метод 1: Прямая команда
    Commands::send_servo_direct(32, pos);
    delay(1000);
    
    // Метод 2: Через SafetySystem
    SafetySystem::set_servo(32, pos);
    delay(1000);
    
    // Возврат в нейтраль
    Commands::send_servo_direct(32, neutral);
    delay(1000);
  }
  
  Logger::log(Logger::INFO, "=== ДИАГНОСТИКА SERVO 32 ЗАВЕРШЕНА ===");
  
  // Финальный сброс всей системы
  Commands::reset_all_servos();
}

void test_joint_directions() {
  Logger::log(Logger::INFO, "Starting ULTRA-STABLE joint direction test");
  is_moving = false;
  
  const char* leg_names[] = {"FR", "MR", "RR", "RL", "ML", "FL"};
  const char* joint_names[] = {"COXA", "FEMUR", "TIBIA"};
  const int TEST_OFFSET = 60; // Безопасное смещение с новыми пределами PWM 1000-2000
  
  // ЭКСТРЕМАЛЬНАЯ стабилизация: полная остановка и сброс
  Logger::log(Logger::INFO, "=== ULTRA-STABILIZATION SEQUENCE ===");
  
  // 1. Аварийная остановка всех сервоприводов
  SafetySystem::emergency_stop();
  delay(2000);
  
  // 2. Принудительный сброс каждого сервопривода в базовый нейтраль
  Logger::log(Logger::INFO, "Force-resetting each servo to base NEUTRAL");
  for (int servo = 1; servo <= 32; servo++) {
    Commands::send_servo_direct(servo, NEUTRAL);
    delay(50); // Небольшая задержка между сервоприводами
  }
  delay(3000);
  
  // 3. Установка всех сервоприводов в расчетные нейтральные позиции
  Logger::log(Logger::INFO, "Setting all servos to calculated neutral positions");
  for (int leg = 0; leg < TOTAL_LEGS; leg++) {
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
      int servo = LEG_SERVO_MAP[leg][joint];
      int neutral = NEUTRAL + LEG_OFFSETS[leg][joint];
      Logger::log(Logger::INFO, "Servo %d (Leg %s %s): %d", 
                 servo, leg_names[leg], joint_names[joint], neutral);
      Commands::send_servo_direct(servo, neutral);
      delay(100);
    }
  }
  
  Logger::log(Logger::INFO, "=== STABILIZATION COMPLETE ===");
  delay(5000); // Долгая пауза для полной стабилизации
  
  // Тестируем каждый тип сустава отдельно
  for (int joint = 0; joint < NUM_JOINTS; joint++) {
    Logger::log(Logger::INFO, "");
    Logger::log(Logger::INFO, "██████ Testing ALL %s joints ██████", joint_names[joint]);
    
    // Предварительный расчет всех позиций с проверкой
    int neutral_positions[TOTAL_LEGS];
    int pos_positions[TOTAL_LEGS]; 
    int neg_positions[TOTAL_LEGS];
    
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      neutral_positions[leg] = NEUTRAL + LEG_OFFSETS[leg][joint];
      
      // ИСПРАВЛЕНИЕ: Левые ноги (RL, ML, FL) получают инвертированные значения
      // чтобы двигаться в ТУ ЖЕ физическую сторону, что и правые
      bool is_left_leg = (leg == LEG_REAR_LEFT || leg == LEG_MIDDLE_LEFT || leg == LEG_FRONT_LEFT);
      
      if (is_left_leg) {
        // Для левых ног инвертируем направление
        pos_positions[leg] = constrain(neutral_positions[leg] - TEST_OFFSET, MIN_PULSE, MAX_PULSE);
        neg_positions[leg] = constrain(neutral_positions[leg] + TEST_OFFSET, MIN_PULSE, MAX_PULSE);
      } else {
        // Для правых ног нормальное направление
        pos_positions[leg] = constrain(neutral_positions[leg] + TEST_OFFSET, MIN_PULSE, MAX_PULSE);
        neg_positions[leg] = constrain(neutral_positions[leg] - TEST_OFFSET, MIN_PULSE, MAX_PULSE);
      }
      
      Logger::log(Logger::INFO, "Leg %s (%s): N=%d, (+)=%d, (-)=%d", 
                 leg_names[leg], is_left_leg ? "LEFT" : "RIGHT",
                 neutral_positions[leg], pos_positions[leg], neg_positions[leg]);
    }
    
    // === ПОЛОЖИТЕЛЬНОЕ ДВИЖЕНИЕ ===
    Logger::log(Logger::INFO, "▶▶▶ Moving all %s joints to POSITIVE ▶▶▶", joint_names[joint]);
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      int servo = LEG_SERVO_MAP[leg][joint];
      Commands::send_servo_direct(servo, pos_positions[leg]);
      delay(50); // Задержка между сервоприводами
    }
    delay(3000); // Долгое время для движения и наблюдения
    
    // === ВОЗВРАТ В НЕЙТРАЛЬ ===
    Logger::log(Logger::INFO, "◄─► Returning all %s joints to NEUTRAL ◄─►", joint_names[joint]);
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      int servo = LEG_SERVO_MAP[leg][joint];
      Commands::send_servo_direct(servo, neutral_positions[leg]);
      delay(50);
    }
    delay(2000);
    
    // === ОТРИЦАТЕЛЬНОЕ ДВИЖЕНИЕ ===
    Logger::log(Logger::INFO, "◄◄◄ Moving all %s joints to NEGATIVE ◄◄◄", joint_names[joint]);
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      int servo = LEG_SERVO_MAP[leg][joint];
      Commands::send_servo_direct(servo, neg_positions[leg]);
      delay(50);
    }
    delay(3000);
    
    // === ФИНАЛЬНЫЙ ВОЗВРАТ ===
    Logger::log(Logger::INFO, "◄─► Final return of all %s joints to NEUTRAL ◄─►", joint_names[joint]);
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
      int servo = LEG_SERVO_MAP[leg][joint];
      Commands::send_servo_direct(servo, neutral_positions[leg]);
      delay(50);
    }
    
    Logger::log(Logger::INFO, "██████ %s joints test COMPLETE ██████", joint_names[joint]);
    delay(4000); // Долгая пауза между типами суставов
  }
  
  Logger::log(Logger::INFO, "=== ULTRA-STABLE test completed - final system reset ===");
  Commands::reset_all_servos();
}

void test_tripod_gait() {
  Logger::log(Logger::INFO, "Starting SAFE tripod gait with CONSERVATIVE amplitude");
  is_moving = false; // Остановить обычное движение
  
  const char* leg_names[] = {"FR", "MR", "RR", "RL", "ML", "FL"};
  const int LIFT_AMOUNT = 40;  // БЕЗОПАСНАЯ величина подъема (было 120!)
  const int FORWARD_AMOUNT = 30; // БЕЗОПАСНАЯ величина движения вперед (было 80!)
  
  Logger::log(Logger::INFO, "Using lift: %d, forward: %d", LIFT_AMOUNT, FORWARD_AMOUNT);
  
  // Сначала установить всех в стартовую позицию
  Logger::log(Logger::INFO, "Setting all legs to startup position");
  Commands::reset_all_servos();
  delay(2000);
  
  // Тест полной трипоидной походки с движением вперед
  for (int cycle = 0; cycle < 2; cycle++) {
    Logger::log(Logger::INFO, "Enhanced tripod cycle %d", cycle + 1);
    
    // === ФАЗА 1: Группа 1 в воздухе, движется вперед ===
    Logger::log(Logger::INFO, "PHASE 1: Group 1 (FR,ML,RR) lifting and moving forward");
    int group1[] = {LEG_FRONT_RIGHT, LEG_MIDDLE_LEFT, LEG_REAR_RIGHT};
    int group2[] = {LEG_FRONT_LEFT, LEG_MIDDLE_RIGHT, LEG_REAR_LEFT};
    
    // Поднимаем группу 1 и двигаем вперед
    for (int i = 0; i < 3; i++) {
      int leg = group1[i];
      
      // Простая логика без лишних инверсий - таблица LEG_LIFT_DIRECTIONS уже правильная
      int coxa_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][COXA] + FORWARD_AMOUNT, 
        MIN_PULSE, MAX_PULSE);
      
      int femur_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][FEMUR] + (LEG_LIFT_DIRECTIONS[leg][FEMUR] * LIFT_AMOUNT), 
        MIN_PULSE, MAX_PULSE);
      
      int tibia_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][TIBIA] + (LEG_LIFT_DIRECTIONS[leg][TIBIA] * LIFT_AMOUNT), 
        MIN_PULSE, MAX_PULSE);
      
      Logger::log(Logger::INFO, "Group1 %s: COXA=%d (+%d), FEMUR=%d (%+d), TIBIA=%d (%+d)", 
                 leg_names[leg], coxa_pulse, FORWARD_AMOUNT,
                 femur_pulse, LEG_LIFT_DIRECTIONS[leg][FEMUR] * LIFT_AMOUNT,
                 tibia_pulse, LEG_LIFT_DIRECTIONS[leg][TIBIA] * LIFT_AMOUNT);
      
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], coxa_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], femur_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], tibia_pulse);
    }
    
    // Одновременно группа 2 на земле толкает назад
    for (int i = 0; i < 3; i++) {
      int leg = group2[i];
      
      int coxa_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][COXA] - FORWARD_AMOUNT, 
        MIN_PULSE, MAX_PULSE);
      
      int femur_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][FEMUR], 
        MIN_PULSE, MAX_PULSE);
      
      int tibia_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][TIBIA], 
        MIN_PULSE, MAX_PULSE);
      
      Logger::log(Logger::INFO, "Group2 %s: COXA=%d (-%d), ground support", 
                 leg_names[leg], coxa_pulse, FORWARD_AMOUNT);
      
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], coxa_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], femur_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], tibia_pulse);
    }
    
    delay(2000); // Время для выполнения движения
    
    // Опускаем группу 1 
    Logger::log(Logger::INFO, "Lowering group 1 to ground");
    for (int i = 0; i < 3; i++) {
      int leg = group1[i];
      
      int coxa_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][COXA] + FORWARD_AMOUNT, 
        MIN_PULSE, MAX_PULSE);
      
      int femur_pulse = constrain(NEUTRAL + LEG_OFFSETS[leg][FEMUR], MIN_PULSE, MAX_PULSE);
      int tibia_pulse = constrain(NEUTRAL + LEG_OFFSETS[leg][TIBIA], MIN_PULSE, MAX_PULSE);
      
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], coxa_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], femur_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], tibia_pulse);
    }
    delay(1000);
    
    // === ФАЗА 2: Группа 2 в воздухе, движется вперед ===
    Logger::log(Logger::INFO, "PHASE 2: Group 2 (FL,MR,RL) lifting and moving forward");
    
    // Поднимаем группу 2 и двигаем вперед
    for (int i = 0; i < 3; i++) {
      int leg = group2[i];
      
      int coxa_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][COXA] + FORWARD_AMOUNT, 
        MIN_PULSE, MAX_PULSE);
      
      int femur_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][FEMUR] + (LEG_LIFT_DIRECTIONS[leg][FEMUR] * LIFT_AMOUNT), 
        MIN_PULSE, MAX_PULSE);
      
      int tibia_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][TIBIA] + (LEG_LIFT_DIRECTIONS[leg][TIBIA] * LIFT_AMOUNT), 
        MIN_PULSE, MAX_PULSE);
      
      Logger::log(Logger::INFO, "Group2 %s: COXA=%d (+%d), FEMUR=%d (%+d), TIBIA=%d (%+d)", 
                 leg_names[leg], coxa_pulse, FORWARD_AMOUNT,
                 femur_pulse, LEG_LIFT_DIRECTIONS[leg][FEMUR] * LIFT_AMOUNT,
                 tibia_pulse, LEG_LIFT_DIRECTIONS[leg][TIBIA] * LIFT_AMOUNT);
      
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], coxa_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], femur_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], tibia_pulse);
    }
    
    // Группа 1 на земле толкает назад
    for (int i = 0; i < 3; i++) {
      int leg = group1[i];
      
      int coxa_pulse = constrain(
        NEUTRAL + LEG_OFFSETS[leg][COXA] - FORWARD_AMOUNT, 
        MIN_PULSE, MAX_PULSE);
      
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], coxa_pulse);
    }
    
    delay(2000);
    
    // Опускаем группу 2
    Logger::log(Logger::INFO, "Lowering group 2 to ground");
    for (int i = 0; i < 3; i++) {
      int leg = group2[i];
      
      int femur_pulse = constrain(NEUTRAL + LEG_OFFSETS[leg][FEMUR], MIN_PULSE, MAX_PULSE);
      int tibia_pulse = constrain(NEUTRAL + LEG_OFFSETS[leg][TIBIA], MIN_PULSE, MAX_PULSE);
      
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], femur_pulse);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], tibia_pulse);
    }
    delay(1000);
  }
  
  Logger::log(Logger::INFO, "Enhanced tripod gait test completed - returning to neutral");
  Commands::reset_all_servos();
}

void calibrate_servos() {
  Logger::log(Logger::INFO, "Calibration started");
  for (int leg = 0; leg < TOTAL_LEGS; leg++) {
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
      int servo = LEG_SERVO_MAP[leg][joint];
      SafetySystem::set_servo(servo, NEUTRAL);
    }
  }
}
