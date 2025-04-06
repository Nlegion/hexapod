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

bool is_moving = false;
unsigned long step_start_time = 0;
//int current_leg = 0; // Начинаем с ноги 0
LegID current_leg = LEG_FRONT_RIGHT;

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

    hexapod.reset_pose(current_leg); // Теперь hexapod объявлен
    Logger::log(Logger::INFO, "Ready. All servos in neutral position");
}

void loop() {
    webSocket.loop();
    server.handleClient();
    SafetySystem::update_load_monitor();

    if(is_moving) {
        hexapod.update_single_leg(static_cast<LegID>(current_leg), millis() - step_start_time);

        // Сбрасываем анимацию через STEP_DURATION
        if(millis() - step_start_time > STEP_DURATION * 1000) {
            is_moving = false;
            hexapod.reset_pose(current_leg);
        }
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

void handle_command(const char* cmd) {
    if(strcmp(cmd, "FWD") == 0) {
        current_leg = LEG_FRONT_RIGHT;
        start_movement();
        return; 
    }
    else if(strcmp(cmd, "CALIBRATE") == 0) {
        calibrate_servos();
    }
    else if(strcmp(cmd, "STOP") == 0) {
        Logger::log(Logger::INFO, "Executing STOP command");
        is_moving = false;
        hexapod.reset_pose(current_leg);
        return;
    }

    Logger::log(Logger::INFO, "Command '%s' received (not implemented yet)", cmd);
}

void start_movement() {
    is_moving = true;
    step_start_time = millis();
    Logger::log(Logger::INFO, "Starting movement for leg %d", current_leg);
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
