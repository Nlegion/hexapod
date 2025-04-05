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
int current_leg = 0; // Начинаем с ноги 0

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
    
    hexapod.reset_pose(); // Теперь hexapod объявлен
    Logger::log(Logger::INFO, "Ready. All servos in neutral position");
}

void loop() {
    webSocket.loop();
    server.handleClient();
    SafetySystem::update_load_monitor();

    if(is_moving) {
        hexapod.update_single_leg(current_leg, millis() - step_start_time);
        
        // Сбрасываем анимацию через STEP_DURATION
        if(millis() - step_start_time > STEP_DURATION * 1000) {
            is_moving = false;
            hexapod.reset_pose();
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
    if(strcmp(cmd, "STOP") == 0) {
        Logger::log(Logger::INFO, "Executing STOP command");
        is_moving = false;
        hexapod.reset_pose();
        return;
    }
    else if(strcmp(cmd, "FWD") == 0) {
        Logger::log(Logger::INFO, "Starting leg movement");
        is_moving = true;
        step_start_time = millis();
        current_leg = 0; // Начинаем с первой ноги
        return;
    }
    
    Logger::log(Logger::INFO, "Command '%s' received (not implemented yet)", cmd);
}
