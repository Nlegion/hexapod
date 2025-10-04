#pragma once
#include <WebServer.h>
#include "WebSocketController.h"
#include "../../application/RobotController.h"
#include "../../core/Logger.h"
#include <memory>

// ═══════════════════════════════════════════════════════════════
// WEB CONTROLLER
// Управление HTTP сервером и маршрутами
// ═══════════════════════════════════════════════════════════════

namespace Presentation {

class WebController {
public:
    WebController(
        WebServer& server,
        std::shared_ptr<Application::RobotController> robot,
        const char* htmlPage
    ) : server_(server), robot_(robot), htmlPage_(htmlPage) {}

    void initialize() {
        setupRoutes();
        server_.begin();
        Core::Logger::log(Core::Logger::INFO, "✅ WebController initialized on port 80");
    }

    void handleClients() {
        server_.handleClient();
    }

private:
    WebServer& server_;
    std::shared_ptr<Application::RobotController> robot_;
    const char* htmlPage_;

    void setupRoutes() {
        // Главная страница
        server_.on("/", [this]() {
            this->handleRoot();
        });
        
        // API endpoint для статуса
        server_.on("/api/status", [this]() {
            this->handleApiStatus();
        });

        // API endpoint для команды
        server_.on("/api/command", HTTP_POST, [this]() {
            this->handleApiCommand();
        });

        // 404 handler
        server_.onNotFound([this]() {
            this->handleNotFound();
        });
    }

    void handleRoot() {
        server_.send_P(200, "text/html", htmlPage_);
        Core::Logger::log(Core::Logger::INFO, "Served root page");
    }

    void handleApiStatus() {
        const char* status = robot_->getStatusJSON();
        server_.send(200, "application/json", status);
    }

    void handleApiCommand() {
        if (server_.hasArg("cmd")) {
            String command = server_.arg("cmd");
            Core::Logger::log(Core::Logger::INFO, 
                "API command received: %s", command.c_str());
            
            robot_->handleCommand(command.c_str());
            
            const char* status = robot_->getStatusJSON();
            server_.send(200, "application/json", status);
        } else {
            server_.send(400, "application/json", 
                "{\"error\":\"Missing 'cmd' parameter\"}");
        }
    }

    void handleNotFound() {
        server_.send(404, "text/plain", "404: Not Found");
    }
};

} // namespace Presentation

