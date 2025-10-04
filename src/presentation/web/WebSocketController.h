#pragma once
#include <WebSocketsServer.h>
#include "../../application/RobotController.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// WEBSOCKET CONTROLLER
// Обработка WebSocket соединений и команд
// ═══════════════════════════════════════════════════════════════

namespace Presentation {

class WebSocketController {
public:
    WebSocketController(
        WebSocketsServer& webSocket,
        std::shared_ptr<Application::RobotController> robot
    ) : webSocket_(webSocket), robot_(robot) {}

    void initialize() {
        // Устанавливаем callback для событий WebSocket
        webSocket_.begin();
        
        // Важно: callback должен быть static или global
        // Поэтому используем lambda с capture контроллера
        webSocket_.onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
            this->handleEvent(num, type, payload, length);
        });
        
        Core::Logger::log(Core::Logger::INFO, "✅ WebSocketController initialized");
    }

    void handleEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        switch (type) {
            case WStype_DISCONNECTED:
                handleDisconnect(num);
                break;
                
            case WStype_CONNECTED:
                handleConnect(num);
                break;
                
            case WStype_TEXT:
                handleTextMessage(num, payload, length);
                break;
                
            default:
                break;
        }
    }

    void broadcastStatus() {
        const char* status = robot_->getStatusJSON();
        webSocket_.broadcastTXT(status);
    }

    void sendBatteryStatus(const char* batteryData) {
        webSocket_.broadcastTXT(batteryData);
    }

    void loop() {
        webSocket_.loop();
    }

private:
    WebSocketsServer& webSocket_;
    std::shared_ptr<Application::RobotController> robot_;

    void handleDisconnect(uint8_t num) {
        Core::Logger::log(Core::Logger::INFO, 
            "WebSocket [%u] disconnected", num);
    }

    void handleConnect(uint8_t num) {
        IPAddress ip = webSocket_.remoteIP(num);
        Core::Logger::log(Core::Logger::INFO, 
            "WebSocket [%u] connected from %d.%d.%d.%d",
            num, ip[0], ip[1], ip[2], ip[3]);
        
        // Отправляем текущий статус
        const char* status = robot_->getStatusJSON();
        webSocket_.sendTXT(num, status);
    }

    void handleTextMessage(uint8_t num, uint8_t* payload, size_t length) {
        String command = String((char*)payload);
        Core::Logger::log(Core::Logger::INFO, 
            "WebSocket [%u] command: %s", num, command.c_str());
        
        // Обрабатываем команду через RobotController
        robot_->handleCommand(command.c_str());
        
        // Отправляем обновленный статус
        const char* status = robot_->getStatusJSON();
        webSocket_.sendTXT(num, status);
    }
};

} // namespace Presentation

