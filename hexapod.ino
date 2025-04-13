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

const LegPosition tripod_positions[2][TOTAL_LEGS] = {
    // Phase 1
    {
        // LEG_FRONT_RIGHT (45°)
        { 45.0f, 35.0f, 30.0f, 700 },   
        // LEG_MIDDLE_RIGHT
        { 0.0f, 50.0f, -40.0f, 500 },
        // LEG_REAR_RIGHT 
        { -35.0f, 35.0f, 30.0f, 700 },
        // LEG_REAR_LEFT
        { -45.0f, 35.0f, 30.0f, 700 },
        // LEG_MIDDLE_LEFT
        { 0.0f, 50.0f, -40.0f, 700 },
        // LEG_FRONT_LEFT
        { 35.0f, 35.0f, 30.0f, 700 }
    },
    // Phase 2
    {
        // LEG_FRONT_RIGHT 
        { 35.0f, 50.0f, -40.0f, 700 },   
        // LEG_MIDDLE_RIGHT 
        { -35.0f, 35.0f, 30.0f, 700 },   
        // LEG_REAR_RIGHT 
        { -45.0f, 50.0f, -40.0f, 700 },   
        // LEG_REAR_LEFT 
        { 45.0f, 50.0f, -40.0f, 700 },    
        // LEG_MIDDLE_LEFT 
        { 35.0f, 35.0f, 30.0f, 700 },    
        // LEG_FRONT_LEFT 
        { -35.0f, 50.0f, -40.0f, 700 }    
    }
};

bool is_moving = false;
unsigned long step_start_time = 0;

void init_webserver();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
void handle_command(const char* cmd);
enum class GaitState {
    IDLE,
    LIFT,
    MOVE,
    LOWER
};
enum class GaitPhase {
    PHASE1, // Первая тройка ног в воздухе
    PHASE2  // Вторая тройка ног в воздухе
};

GaitPhase current_phase = GaitPhase::PHASE1;
unsigned long phase_start_time = 0;
GaitState gait_state = GaitState::IDLE;
LegID active_leg = LEG_FRONT_RIGHT;
float progress = 0.0f;

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
     for(int leg = 0; leg < TOTAL_LEGS; leg++) {
        hexapod.set_target_position(
            static_cast<LegID>(leg),
            tripod_positions[0][leg]
        );
    }

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
    if(!is_moving) return;

    if(millis() - phase_start_time > GAIT_DELAY * 2) {
        // Переключение фазы
        current_phase = (current_phase == GaitPhase::PHASE1) ? 
            GaitPhase::PHASE2 : GaitPhase::PHASE1;
        
        // Установка новых целей
        for(int leg = 0; leg < TOTAL_LEGS; leg++) {
            hexapod.set_target_position(
                static_cast<LegID>(leg),
                tripod_positions[static_cast<int>(current_phase)][leg]
            );
        }
        phase_start_time = millis();
    }

    hexapod.update_all_legs();
}



void handle_command(const char* cmd) {
    if(strcmp(cmd, "FWD") == 0) {
        is_moving = true;
        current_phase = GaitPhase::PHASE1;
        phase_start_time = millis();
        Logger::log(Logger::INFO, "Start tripod gait forward");
    }
    else if(strcmp(cmd, "STOP") == 0) {
        Logger::log(Logger::INFO, "Executing STOP command");
        is_moving = false;
        // Возврат в нейтральное положение
        for(int leg = 0; leg < TOTAL_LEGS; leg++) {
            hexapod.reset_pose(static_cast<LegID>(leg));
        }
    }
    else if(strcmp(cmd, "CALIBRATE") == 0) {
    Commands::calibration_mode();
}
    else {
        Logger::log(Logger::WARNING, "Unknown command: %s", cmd);
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch(type) {
        case WStype_CONNECTED:
            Logger::log(Logger::INFO, "Client %d connected", num);
            break;

        case WStype_TEXT: {
            String cmd((char*)payload);
            Logger::log(Logger::INFO, "Received command: %s", cmd.c_str());
            handle_command(cmd.c_str());
            break;
        }

        case WStype_DISCONNECTED:
            Logger::log(Logger::INFO, "Client %d disconnected", num);
            break;
    }
}

void start_movement() {
    is_moving = true;
    step_start_time = millis();
    Logger::log(Logger::INFO, "Starting movement for leg %d", static_cast<int>(active_leg));
}

void calibrate_servos() {
    Logger::log(Logger::INFO, "Calibration started");
    for(int leg = 0; leg < TOTAL_LEGS; leg++) {
        for(int joint = 0; joint < NUM_JOINTS; joint++) {
            int servo = LEG_SERVO_MAP[leg][joint];
            SafetySystem::set_servo(servo, NEUTRAL);
        }
    }
}
