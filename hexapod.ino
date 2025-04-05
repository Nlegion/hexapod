#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "logger.h"
#include "page_html.h"
#include "safety.h"
#include "commands.h"
#include "kinematics.h" // Добавляем явное подключение kinematics

WebServer server(80);
WebSocketsServer webSocket(81);
LegController hexapod; // Теперь компилятор видит объявление из kinematics.h

void init_webserver();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
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
        hexapod.reset_pose();
        return;
    }
    
    // Для всех остальных команд просто выводим сообщение
    Logger::log(Logger::INFO, "Command '%s' received (not implemented yet)", cmd);
}
