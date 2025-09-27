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
  } else if (strcmp(cmd, "TRIPOD_INVERT") == 0) {
    test_tripod_with_inverted_left_legs();
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

void test_tripod_with_inverted_left_legs() {
  Logger::log(Logger::INFO, "Starting INVERTED LEFT LEGS tripod test");
  is_moving = false;
  
  const char* leg_names[] = {"FR", "MR", "RR", "RL", "ML", "FL"};
  const int LIFT_AMOUNT = 40;
  const int FORWARD_AMOUNT = 30;
  
  Logger::log(Logger::INFO, "Testing with INVERTED logic for left legs");
  
  Commands::reset_all_servos();
  delay(2000);
  
  for (int cycle = 0; cycle < 1; cycle++) {
    Logger::log(Logger::INFO, "Inverted test cycle %d", cycle + 1);
    
    // === ФАЗА 1: FR, ML, RR поднимаются (ML с инверсией!) ===
    Logger::log(Logger::INFO, "PHASE 1: FR normal, ML INVERTED, RR normal");
    
    // FR (Front Right) - нормально
    int fr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int fr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE);
    int fr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE);
    
    // ML (Middle Left) - ИНВЕРТИРОВАННАЯ логика
    int ml_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int ml_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // ИНВЕРСИЯ!
    int ml_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // ИНВЕРСИЯ!
    
    // RR (Rear Right) - нормально  
    int rr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE);
    
    Logger::log(Logger::INFO, "FR: COXA=%d, FEMUR=%d (+%d), TIBIA=%d (-%d)", fr_coxa, fr_femur, LIFT_AMOUNT, fr_tibia, LIFT_AMOUNT);
    Logger::log(Logger::INFO, "ML: COXA=%d, FEMUR=%d (-%d), TIBIA=%d (+%d) [INVERTED]", ml_coxa, ml_femur, LIFT_AMOUNT, ml_tibia, LIFT_AMOUNT);
    Logger::log(Logger::INFO, "RR: COXA=%d, FEMUR=%d (+%d), TIBIA=%d (-%d)", rr_coxa, rr_femur, LIFT_AMOUNT, rr_tibia, LIFT_AMOUNT);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][COXA], fr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][FEMUR], fr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][TIBIA], fr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][COXA], ml_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][FEMUR], ml_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][TIBIA], ml_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][COXA], rr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][FEMUR], rr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][TIBIA], rr_tibia);
    
    delay(4000); // Долго для наблюдения
    
    // Сброс
    Logger::log(Logger::INFO, "Resetting all to neutral");
    Commands::reset_all_servos();
    delay(2000);
    
    // === ФАЗА 2: FL, MR, RL (FL и RL с инверсией!) ===
    Logger::log(Logger::INFO, "PHASE 2: FL INVERTED, MR normal, RL INVERTED");
    
    // FL (Front Left) - ИНВЕРТИРОВАННАЯ логика  
    int fl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int fl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // ИНВЕРСИЯ!
    int fl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][TIBIA] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // ИНВЕРСИЯ!
    
    // MR (Middle Right) - нормально
    int mr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int mr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE);
    int mr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE);
    
    // RL (Rear Left) - ИНВЕРТИРОВАННАЯ логика
    int rl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // ИНВЕРСИЯ!
    int rl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][TIBIA] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // ИНВЕРСИЯ!
    
    Logger::log(Logger::INFO, "FL: COXA=%d, FEMUR=%d (-%d), TIBIA=%d (+%d) [INVERTED]", fl_coxa, fl_femur, LIFT_AMOUNT, fl_tibia, LIFT_AMOUNT);
    Logger::log(Logger::INFO, "MR: COXA=%d, FEMUR=%d (+%d), TIBIA=%d (-%d)", mr_coxa, mr_femur, LIFT_AMOUNT, mr_tibia, LIFT_AMOUNT);
    Logger::log(Logger::INFO, "RL: COXA=%d, FEMUR=%d (-%d), TIBIA=%d (+%d) [INVERTED]", rl_coxa, rl_femur, LIFT_AMOUNT, rl_tibia, LIFT_AMOUNT);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][COXA], fl_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][FEMUR], fl_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][TIBIA], fl_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][COXA], mr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][FEMUR], mr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][TIBIA], mr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][COXA], rl_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][FEMUR], rl_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][TIBIA], rl_tibia);
    
    delay(4000); // Долго для наблюдения
  }
  
  Logger::log(Logger::INFO, "Inverted left legs test completed");
  Commands::reset_all_servos();
}

void test_tripod_gait() {
  Logger::log(Logger::INFO, "Starting EXPLICIT tripod gait with individual leg control");
  is_moving = false;
  
  const char* leg_names[] = {"FR", "MR", "RR", "RL", "ML", "FL"};
  const int LIFT_AMOUNT = 40;
  const int FORWARD_AMOUNT = 30;
  
  Logger::log(Logger::INFO, "Using lift: %d, forward: %d", LIFT_AMOUNT, FORWARD_AMOUNT);
  
  // Стартовая позиция
  Commands::reset_all_servos();
  delay(2000);
  
  for (int cycle = 0; cycle < 2; cycle++) {
    Logger::log(Logger::INFO, "Tripod cycle %d", cycle + 1);
    
    // === ФАЗА 1: FR, ML, RR поднимаются ===
    Logger::log(Logger::INFO, "PHASE 1: Lifting FR, ML, RR");
    
    // FR (Front Right) - поднимаем
    int fr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int fr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // +LIFT для подъема
    int fr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // -LIFT для сгибания
    
    // ML (Middle Left) - поднимаем (ВНИМАНИЕ: может нуждаться в инверсии)
    int ml_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int ml_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // Попробуем +LIFT
    int ml_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // Попробуем -LIFT
    
    // RR (Rear Right) - поднимаем
    int rr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // +LIFT для подъема
    int rr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // -LIFT для сгибания
    
    Logger::log(Logger::INFO, "FR: COXA=%d, FEMUR=%d, TIBIA=%d", fr_coxa, fr_femur, fr_tibia);
    Logger::log(Logger::INFO, "ML: COXA=%d, FEMUR=%d, TIBIA=%d", ml_coxa, ml_femur, ml_tibia);
    Logger::log(Logger::INFO, "RR: COXA=%d, FEMUR=%d, TIBIA=%d", rr_coxa, rr_femur, rr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][COXA], fr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][FEMUR], fr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][TIBIA], fr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][COXA], ml_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][FEMUR], ml_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][TIBIA], ml_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][COXA], rr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][FEMUR], rr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][TIBIA], rr_tibia);
    
    // FL, MR, RL на земле толкают назад
    Logger::log(Logger::INFO, "FL, MR, RL ground support pushing back");
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][COXA], 
        constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE));
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][COXA], 
        constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE));
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][COXA], 
        constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE));
    
    delay(3000); // Дольше для наблюдения
    
    // Опускаем группу 1
    Logger::log(Logger::INFO, "Lowering FR, ML, RR to ground");
    hexapod.reset_pose(static_cast<LegID>(LEG_FRONT_RIGHT));
    hexapod.reset_pose(static_cast<LegID>(LEG_MIDDLE_LEFT));
    hexapod.reset_pose(static_cast<LegID>(LEG_REAR_RIGHT));
    delay(1000);
    
    // === ФАЗА 2: FL, MR, RL поднимаются ===
    Logger::log(Logger::INFO, "PHASE 2: Lifting FL, MR, RL");
    
    // FL (Front Left) - поднимаем (ВНИМАНИЕ: может нуждаться в инверсии)
    int fl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int fl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // Попробуем +LIFT
    int fl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // Попробуем -LIFT
    
    // MR (Middle Right) - поднимаем
    int mr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int mr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // +LIFT для подъема
    int mr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // -LIFT для сгибания
    
    // RL (Rear Left) - поднимаем (ВНИМАНИЕ: может нуждаться в инверсии)
    int rl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // Попробуем +LIFT
    int rl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // Попробуем -LIFT
    
    Logger::log(Logger::INFO, "FL: COXA=%d, FEMUR=%d, TIBIA=%d", fl_coxa, fl_femur, fl_tibia);
    Logger::log(Logger::INFO, "MR: COXA=%d, FEMUR=%d, TIBIA=%d", mr_coxa, mr_femur, mr_tibia);
    Logger::log(Logger::INFO, "RL: COXA=%d, FEMUR=%d, TIBIA=%d", rl_coxa, rl_femur, rl_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][COXA], fl_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][FEMUR], fl_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][TIBIA], fl_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][COXA], mr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][FEMUR], mr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][TIBIA], mr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][COXA], rl_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][FEMUR], rl_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][TIBIA], rl_tibia);
    
    // FR, ML, RR на земле толкают назад
    Logger::log(Logger::INFO, "FR, ML, RR ground support pushing back");
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][COXA], 
        constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE));
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][COXA], 
        constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE));
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][COXA], 
        constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE));
    
    delay(3000); // Дольше для наблюдения
    
    // Опускаем группу 2
    Logger::log(Logger::INFO, "Lowering FL, MR, RL to ground");
    hexapod.reset_pose(static_cast<LegID>(LEG_FRONT_LEFT));
    hexapod.reset_pose(static_cast<LegID>(LEG_MIDDLE_RIGHT));
    hexapod.reset_pose(static_cast<LegID>(LEG_REAR_LEFT));
    delay(1000);
  }
  
  Logger::log(Logger::INFO, "Explicit tripod test completed");
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
