#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "logger.h"
#include "page_html.h"
#include "safety.h"
#include "commands.h"
#include "kinematics.h"

// Определение статических переменных ControllerStatus
bool ControllerStatus::initialized = false;
unsigned long ControllerStatus::last_command_time = 0;
int ControllerStatus::command_count = 0;

WebServer server(80);
WebSocketsServer webSocket(81);
LegController hexapod;

// Добавляем переменные для управления шагами
int current_step = 0;
unsigned long last_step_time = 0;

// Оптимизированные параметры походки с адаптивной скоростью
const unsigned long STEP_DELAY = 150; // Оптимизированный интервал для плавности
const unsigned long FAST_STEP_DELAY = 120; // Быстрая походка
const unsigned long SLOW_STEP_DELAY = 200; // Медленная походка
unsigned long current_step_delay = STEP_DELAY; // Текущая скорость

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

// Переменные для неблокирующей инициализации сервоприводов
bool servos_reset = false;
unsigned long setup_complete_time = 0;

void init_webserver();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
void handle_command(const char* cmd);
void sendCoordinatesToWeb(const char* legName, const char* type, int coxa, int femur, int tibia);
void sendPhaseToWeb(const char* phaseInfo);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 4, 5);
  
  // Инициализируем контроллер и очищаем буферы
  delay(1000); // Пауза для стабилизации соединения с контроллером
  

  CommandResult init_result = Commands::init_controller();
  if (init_result != CommandResult::SUCCESS) {
    Logger::log(Logger::ERROR, "Failed to initialize servo controller. Result: %d", (int)init_result);
    Logger::log(Logger::WARNING, "TEMPORARY: Continuing without servo controller for web interface testing");
  } else {
    Logger::log(Logger::INFO, "Servo controller initialized successfully");
  }

  WiFi.begin(SSID, PASSWORD);
  unsigned long wifi_start = millis();
  const unsigned long WIFI_TIMEOUT = 15000; // 15 секунд на подключение
  
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifi_start > WIFI_TIMEOUT) {
      Logger::log(Logger::ERROR, "WiFi connection timeout. Starting in AP mode");
      WiFi.mode(WIFI_AP);
      WiFi.softAP("Hexapod_Config", "12345678");
      Logger::log(Logger::INFO, "AP Mode. IP: %s", WiFi.softAPIP().toString().c_str());
      break;
    }
    delay(500);
    Logger::log(Logger::INFO, "Connecting to WiFi...");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Logger::log(Logger::INFO, "Connected. IP: %s", WiFi.localIP().toString().c_str());
  }

  server.on("/", []() {
    server.send_P(200, "text/html", PAGE_HTML);
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  SafetySystem::init();
  
  // Инициализация системы кинематики
  hexapod.init();
  
  // Неблокирующая инициализация - сервоприводы будут сброшены в первом цикле loop()
  setup_complete_time = millis();
  
  Logger::log(Logger::INFO, "Setup complete. Servos will be reset in main loop");
}

void loop() {
  // Неблокирующая инициализация сервоприводов после setup()
  if (!servos_reset && millis() - setup_complete_time > 1000) {
    Commands::reset_all_servos();
    servos_reset = true;
    Logger::log(Logger::INFO, "Ready. All servos in neutral position");
  }
  
  // Мониторинг WiFi соединения
  static unsigned long last_wifi_check = 0;
  if (millis() - last_wifi_check > 5000) { // Проверяем каждые 5 секунд
    last_wifi_check = millis();
    if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) {
      Logger::log(Logger::WARNING, "WiFi disconnected. Attempting reconnection...");
      WiFi.reconnect();
    }
  }
  
  webSocket.loop();
  server.handleClient();
  SafetySystem::update_load_monitor();

  handle_gait_cycle();
}

void handle_gait_cycle() {
  if (!is_moving) return;

  if (millis() - last_step_time >= current_step_delay) {
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

    // НОВАЯ ЛОГИКА: Зеркальные сервоприводы получают те же команды для того же физического движения
    // Применяем LEG_LIFT_DIRECTIONS для всех ног единообразно
    
    int base_coxa_offset = traj[current_step][0] - NEUTRAL;
    int base_femur_offset = traj[current_step][1] - NEUTRAL;
    int base_tibia_offset = traj[current_step][2] - NEUTRAL;
    
    int coxa = NEUTRAL + LEG_OFFSETS[leg][COXA] + (base_coxa_offset * LEG_LIFT_DIRECTIONS[leg][COXA]);
    int femur = NEUTRAL + LEG_OFFSETS[leg][FEMUR] + (base_femur_offset * LEG_LIFT_DIRECTIONS[leg][FEMUR]);
    int tibia = NEUTRAL + LEG_OFFSETS[leg][TIBIA] + (base_tibia_offset * LEG_LIFT_DIRECTIONS[leg][TIBIA]);
      
      // Применяем безопасные ограничения
      coxa = constrain(coxa, MIN_PULSE, MAX_PULSE);
      femur = constrain(femur, MIN_PULSE, MAX_PULSE);
      tibia = constrain(tibia, MIN_PULSE, MAX_PULSE);

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
  } else if (strcmp(cmd, "FAST") == 0) {
    current_step_delay = FAST_STEP_DELAY;
    Logger::log(Logger::INFO, "Fast gait activated (delay: %lu ms)", current_step_delay);
  } else if (strcmp(cmd, "SLOW") == 0) {
    current_step_delay = SLOW_STEP_DELAY;
    Logger::log(Logger::INFO, "Slow gait activated (delay: %lu ms)", current_step_delay);
  } else if (strcmp(cmd, "NORMAL") == 0) {
    current_step_delay = STEP_DELAY;
    Logger::log(Logger::INFO, "Normal gait activated (delay: %lu ms)", current_step_delay);
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
        // Защита от слишком длинных команд
        const size_t MAX_COMMAND_LENGTH = 256;
        if (length > MAX_COMMAND_LENGTH) {
          Logger::log(Logger::WARNING, "Command too long (%d bytes), ignoring", length);
          break;
        }
        
        char cmd[MAX_COMMAND_LENGTH + 1];
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

// Функция для отправки координат на веб-страницу
void sendCoordinatesToWeb(const char* legName, const char* type, int coxa, int femur, int tibia) {
  char coordMsg[200];
  snprintf(coordMsg, sizeof(coordMsg), "COORD:%s:%s:%d:%d:%d", legName, type, coxa, femur, tibia);
  webSocket.broadcastTXT(coordMsg);
}

// Функция для отправки информации о фазе на веб-страницу
void sendPhaseToWeb(const char* phaseInfo) {
  char phaseMsg[100];
  snprintf(phaseMsg, sizeof(phaseMsg), "PHASE:%s", phaseInfo);
  webSocket.broadcastTXT(phaseMsg);
}


void test_joint_directions() {
  Logger::log(Logger::INFO, "Starting ULTRA-STABLE joint direction test");
  is_moving = false;
  
  const char* leg_names[] = {"FR", "MR", "RR", "RL", "ML", "FL"};
  const char* joint_names[] = {"COXA", "FEMUR", "TIBIA"};
  const int TEST_OFFSET = 150; // Увеличено с 60 до 150 для лучшей видимости движений
  
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
      
      // ИСПОЛЬЗУЕМ LEG_LIFT_DIRECTIONS для определения правильного направления
      bool is_left_leg = (leg == LEG_REAR_LEFT || leg == LEG_MIDDLE_LEFT || leg == LEG_FRONT_LEFT);
      
      // Применяем направления согласно LEG_LIFT_DIRECTIONS для данного сустава
      int direction = LEG_LIFT_DIRECTIONS[leg][joint];
      
      pos_positions[leg] = constrain(neutral_positions[leg] + (TEST_OFFSET * direction), MIN_PULSE, MAX_PULSE);
      neg_positions[leg] = constrain(neutral_positions[leg] - (TEST_OFFSET * direction), MIN_PULSE, MAX_PULSE);
      
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
  Logger::log(Logger::INFO, "Starting EXPLICIT tripod gait with individual leg control");
  is_moving = false;
  
  const char* leg_names[] = {"FR", "MR", "RR", "RL", "ML", "FL"};
  const int LIFT_AMOUNT = 180;  // УВЕЛИЧЕНО для более высокого подъема (было 140)
  const int FORWARD_AMOUNT = 120; // Широкий шаг для эффективного движения
  
  Logger::log(Logger::INFO, "Using lift: %d, forward: %d", LIFT_AMOUNT, FORWARD_AMOUNT);
  
  // Очищаем дисплей координат на веб-странице
  webSocket.broadcastTXT("CLEAR_COORDS");
  delay(100);
  
  // Диагностика для ML ноги
  sendPhaseToWeb("ML (MIDDLE LEFT) DIAGNOSTIC");
  Logger::log(Logger::INFO, "=== ML (MIDDLE LEFT) DIAGNOSTIC ===");
  Logger::log(Logger::INFO, "NEUTRAL=%d", NEUTRAL);
  Logger::log(Logger::INFO, "ML LEG_OFFSETS: COXA=%d, FEMUR=%d, TIBIA=%d", 
             LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA], LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR], LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA]);
  Logger::log(Logger::INFO, "ML LEG_LIFT_DIRECTIONS: COXA=%d, FEMUR=%d, TIBIA=%d", 
             LEG_LIFT_DIRECTIONS[LEG_MIDDLE_LEFT][COXA], LEG_LIFT_DIRECTIONS[LEG_MIDDLE_LEFT][FEMUR], LEG_LIFT_DIRECTIONS[LEG_MIDDLE_LEFT][TIBIA]);
  Logger::log(Logger::INFO, "ML SERVO CHANNELS: COXA=%d, FEMUR=%d, TIBIA=%d", 
             LEG_SERVO_MAP[LEG_MIDDLE_LEFT][COXA], LEG_SERVO_MAP[LEG_MIDDLE_LEFT][FEMUR], LEG_SERVO_MAP[LEG_MIDDLE_LEFT][TIBIA]);
  
  // Стартовая позиция
  Commands::reset_all_servos();
  delay(2000);
  
  for (int cycle = 0; cycle < 2; cycle++) {
    Logger::log(Logger::INFO, "Tripod cycle %d", cycle + 1);
    
    // === ФАЗА 1: FR, ML, RR поднимаются ===
    sendPhaseToWeb("PHASE 1: Lifting FR, ML, RR");
    Logger::log(Logger::INFO, "PHASE 1: Lifting FR, ML, RR");
    
    // FR (Front Right) - поднимаем
    Logger::log(Logger::INFO, "=== FR LIFTING CALCULATION (for comparison) ===");
    int fr_coxa_base = NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][COXA];
    int fr_femur_base = NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][FEMUR];  
    int fr_tibia_base = NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][TIBIA];
    Logger::log(Logger::INFO, "FR Base positions: COXA=%d, FEMUR=%d, TIBIA=%d", fr_coxa_base, fr_femur_base, fr_tibia_base);
    
    int fr_coxa = constrain(fr_coxa_base + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int fr_femur = constrain(fr_femur_base + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // +LIFT для подъема
    int fr_tibia = constrain(fr_tibia_base - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // -LIFT для сгибания
    Logger::log(Logger::INFO, "FR Final positions: COXA=%d (%d+80), FEMUR=%d (%d+120), TIBIA=%d (%d-120)", 
               fr_coxa, fr_coxa_base, fr_femur, fr_femur_base, fr_tibia, fr_tibia_base);
    
    // ML (Middle Left) - поднимаем (зеркальный сервопривод - те же команды что у правых!)
    Logger::log(Logger::INFO, "=== ML LIFTING CALCULATION BREAKDOWN ===");
    int ml_coxa_base = NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA];
    int ml_femur_base = NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR];
    int ml_tibia_base = NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA];
    Logger::log(Logger::INFO, "ML Base positions: COXA=%d, FEMUR=%d, TIBIA=%d", ml_coxa_base, ml_femur_base, ml_tibia_base);
    
    int ml_coxa = constrain(ml_coxa_base - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE); // ТОЛЬКО COXA инвертирован для поворота
    int ml_femur = constrain(ml_femur_base + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // FEMUR как у правых - зеркальный сервопривод!
    int ml_tibia = constrain(ml_tibia_base - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // TIBIA как у правых - зеркальный сервопривод!
    Logger::log(Logger::INFO, "ML Final positions: COXA=%d (%d-80), FEMUR=%d (%d+120), TIBIA=%d (%d-120)", 
               ml_coxa, ml_coxa_base, ml_femur, ml_femur_base, ml_tibia, ml_tibia_base);
    
    // RR (Rear Right) - поднимаем
    int rr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // +LIFT для подъема
    int rr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // -LIFT для сгибания
    
    Logger::log(Logger::INFO, "=== PHASE 1 LIFTING LEGS COORDINATES ===");
    Logger::log(Logger::INFO, "FR (Front Right): COXA=%d, FEMUR=%d, TIBIA=%d [LIFTING]", fr_coxa, fr_femur, fr_tibia);
    Logger::log(Logger::INFO, "ML (Middle Left): COXA=%d, FEMUR=%d, TIBIA=%d [LIFTING - INVERTED]", ml_coxa, ml_femur, ml_tibia);
    Logger::log(Logger::INFO, "RR (Rear Right):  COXA=%d, FEMUR=%d, TIBIA=%d [LIFTING]", rr_coxa, rr_femur, rr_tibia);
    
    // Отправляем координаты на веб-страницу
    sendCoordinatesToWeb("FR (Front Right)", "LIFTING", fr_coxa, fr_femur, fr_tibia);
    sendCoordinatesToWeb("ML (Middle Left)", "LIFTING-INVERTED", ml_coxa, ml_femur, ml_tibia);
    sendCoordinatesToWeb("RR (Rear Right)", "LIFTING", rr_coxa, rr_femur, rr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][COXA], fr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][FEMUR], fr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][TIBIA], fr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][COXA], ml_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][FEMUR], ml_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][TIBIA], ml_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][COXA], rr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][FEMUR], rr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][TIBIA], rr_tibia);
    
    // FL, MR, RL на земле толкают назад (левые ноги инвертированы!)
    Logger::log(Logger::INFO, "=== PHASE 1 GROUND SUPPORT LEGS ===");
    
    int fl_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int mr_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rl_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    
    Logger::log(Logger::INFO, "FL (Front Left):   COXA=%d [GROUND - INVERTED PUSH]", fl_ground_coxa);
    Logger::log(Logger::INFO, "MR (Middle Right): COXA=%d [GROUND - NORMAL PUSH]", mr_ground_coxa);
    Logger::log(Logger::INFO, "RL (Rear Left):    COXA=%d [GROUND - INVERTED PUSH]", rl_ground_coxa);
    
    // Отправляем координаты опорных ног (используем нейтральные позиции для FEMUR/TIBIA)
    sendCoordinatesToWeb("FL (Front Left)", "GROUND-INVERTED", fl_ground_coxa, NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR], NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][TIBIA]);
    sendCoordinatesToWeb("MR (Middle Right)", "GROUND", mr_ground_coxa, NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR], NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][TIBIA]);
    sendCoordinatesToWeb("RL (Rear Left)", "GROUND-INVERTED", rl_ground_coxa, NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR], NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][TIBIA]);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][COXA], fl_ground_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][COXA], mr_ground_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][COXA], rl_ground_coxa);
    
    delay(3000); // Дольше для наблюдения
    
    // Опускаем группу 1
    Logger::log(Logger::INFO, "Lowering FR, ML, RR to ground");
    hexapod.reset_pose(static_cast<LegID>(LEG_FRONT_RIGHT));
    hexapod.reset_pose(static_cast<LegID>(LEG_MIDDLE_LEFT));
    hexapod.reset_pose(static_cast<LegID>(LEG_REAR_RIGHT));
    delay(1000);
    
    // === ФАЗА 2: FL, MR, RL поднимаются ===
    sendPhaseToWeb("PHASE 2: Lifting FL, MR, RL");
    Logger::log(Logger::INFO, "PHASE 2: Lifting FL, MR, RL");
    
    // FL (Front Left) - поднимаем (зеркальный сервопривод - те же команды что у правых!)
    int fl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE); // ТОЛЬКО COXA инвертирован для поворота
    int fl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // FEMUR как у правых - зеркальный сервопривод!
    int fl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // TIBIA как у правых - зеркальный сервопривод!
    
    // MR (Middle Right) - поднимаем
    int mr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int mr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // +LIFT для подъема
    int mr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // -LIFT для сгибания
    
    // RL (Rear Left) - поднимаем (зеркальный сервопривод - те же команды что у правых!)
    int rl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE); // ТОЛЬКО COXA инвертирован для поворота
    int rl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // FEMUR как у правых - зеркальный сервопривод!
    int rl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // TIBIA как у правых - зеркальный сервопривод!
    
    Logger::log(Logger::INFO, "=== PHASE 2 LIFTING LEGS COORDINATES ===");
    Logger::log(Logger::INFO, "FL (Front Left):  COXA=%d, FEMUR=%d, TIBIA=%d [LIFTING - INVERTED]", fl_coxa, fl_femur, fl_tibia);
    Logger::log(Logger::INFO, "MR (Middle Right): COXA=%d, FEMUR=%d, TIBIA=%d [LIFTING]", mr_coxa, mr_femur, mr_tibia);
    Logger::log(Logger::INFO, "RL (Rear Left):   COXA=%d, FEMUR=%d, TIBIA=%d [LIFTING - INVERTED]", rl_coxa, rl_femur, rl_tibia);
    
    // Отправляем координаты на веб-страницу
    sendCoordinatesToWeb("FL (Front Left)", "LIFTING-INVERTED", fl_coxa, fl_femur, fl_tibia);
    sendCoordinatesToWeb("MR (Middle Right)", "LIFTING", mr_coxa, mr_femur, mr_tibia);
    sendCoordinatesToWeb("RL (Rear Left)", "LIFTING-INVERTED", rl_coxa, rl_femur, rl_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][COXA], fl_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][FEMUR], fl_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_LEFT][TIBIA], fl_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][COXA], mr_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][FEMUR], mr_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][TIBIA], mr_tibia);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][COXA], rl_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][FEMUR], rl_femur);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_LEFT][TIBIA], rl_tibia);
    
    // FR, ML, RR на земле толкают назад (левые ноги инвертированы!)
    Logger::log(Logger::INFO, "=== PHASE 2 GROUND SUPPORT LEGS ===");
    
    int fr_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int ml_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    int rr_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
    
    Logger::log(Logger::INFO, "FR (Front Right): COXA=%d [GROUND - NORMAL PUSH]", fr_ground_coxa);
    Logger::log(Logger::INFO, "ML (Middle Left): COXA=%d [GROUND - INVERTED PUSH]", ml_ground_coxa);
    Logger::log(Logger::INFO, "RR (Rear Right):  COXA=%d [GROUND - NORMAL PUSH]", rr_ground_coxa);
    
    // Отправляем координаты опорных ног (используем нейтральные позиции для FEMUR/TIBIA)
    sendCoordinatesToWeb("FR (Front Right)", "GROUND", fr_ground_coxa, NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][FEMUR], NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][TIBIA]);
    sendCoordinatesToWeb("ML (Middle Left)", "GROUND-INVERTED", ml_ground_coxa, NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR], NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA]);
    sendCoordinatesToWeb("RR (Rear Right)", "GROUND", rr_ground_coxa, NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][FEMUR], NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][TIBIA]);
    
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_FRONT_RIGHT][COXA], fr_ground_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_MIDDLE_LEFT][COXA], ml_ground_coxa);
    SafetySystem::set_servo(LEG_SERVO_MAP[LEG_REAR_RIGHT][COXA], rr_ground_coxa);
    
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
