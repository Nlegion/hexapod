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
  } else if (strcmp(cmd, "CALIBRATE") == 0) {
    Commands::calibration_mode();
  } else if (strcmp(cmd, "DIAGNOSTIC") == 0) {
    Commands::diagnostic_sequence();
  } else if (strcmp(cmd, "RESET") == 0) {
    Commands::reset_all_servos();
  } else if (strncmp(cmd, "TEST_LEG_", 9) == 0) {
    int leg_id = atoi(cmd + 9); // Извлекаем номер ноги из "TEST_LEG_0"
    Commands::test_single_leg(leg_id);
  } else if (strcmp(cmd, "TRIPOD_TEST") == 0) {
    test_tripod_gait();
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

void test_tripod_gait() {
  Logger::log(Logger::INFO, "Starting tripod gait test");
  is_moving = false; // Остановить обычное движение
  
  // Тест поочередного подъема трипоидных групп
  for (int cycle = 0; cycle < 3; cycle++) {
    Logger::log(Logger::INFO, "Tripod test cycle %d", cycle + 1);
    
    // Фаза 1: Поднимаем группу 1 (FR, ML, RR)
    Logger::log(Logger::INFO, "Lifting tripod group 1 (FR, ML, RR)");
    for (int leg : {LEG_FRONT_RIGHT, LEG_MIDDLE_LEFT, LEG_REAR_RIGHT}) {
      // Поднимаем ногу (уменьшаем tibia, увеличиваем femur)
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], 
        NEUTRAL + LEG_OFFSETS[leg][COXA]);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], 
        NEUTRAL + LEG_OFFSETS[leg][FEMUR] + 200);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], 
        NEUTRAL + LEG_OFFSETS[leg][TIBIA] - 200);
    }
    delay(1500);
    
    // Опускаем обратно
    for (int leg : {LEG_FRONT_RIGHT, LEG_MIDDLE_LEFT, LEG_REAR_RIGHT}) {
      hexapod.reset_pose(static_cast<LegID>(leg));
    }
    delay(1000);
    
    // Фаза 2: Поднимаем группу 2 (FL, MR, RL)  
    Logger::log(Logger::INFO, "Lifting tripod group 2 (FL, MR, RL)");
    for (int leg : {LEG_FRONT_LEFT, LEG_MIDDLE_RIGHT, LEG_REAR_LEFT}) {
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], 
        NEUTRAL + LEG_OFFSETS[leg][COXA]);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], 
        NEUTRAL + LEG_OFFSETS[leg][FEMUR] + 200);
      SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], 
        NEUTRAL + LEG_OFFSETS[leg][TIBIA] - 200);
    }
    delay(1500);
    
    // Опускаем обратно
    for (int leg : {LEG_FRONT_LEFT, LEG_MIDDLE_RIGHT, LEG_REAR_LEFT}) {
      hexapod.reset_pose(static_cast<LegID>(leg));
    }
    delay(1000);
  }
  
  Logger::log(Logger::INFO, "Tripod gait test completed");
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
