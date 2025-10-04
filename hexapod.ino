// ═══════════════════════════════════════════════════════════════
// HEXAPOD ROBOT - Clean Architecture Edition
// 6-Legged Walking Robot with Tripod Gait
// 
// Architecture: Clean Architecture + FreeRTOS
// Hardware: ESP32-S3-DevKitC-1 + 32-channel servo controller
// ═══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// Clean Architecture includes
#include "src/core/Types.h"
#include "src/core/Config.h"
#include "src/core/Logger.h"
#include "src/di/Container.h"
#include "src/application/RobotController.h"

// Старый HTML interface (пока используем старый)
#include "page_html.h"

// ═══════════════════════════════════════════════════════════════
// NETWORK CONFIGURATION
// ═══════════════════════════════════════════════════════════════

const char* SSID = "YOUR_WIFI_SSID";      // Замените на ваш WiFi
const char* PASSWORD = "YOUR_PASSWORD";    // Замените на ваш пароль

// ═══════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════

WebServer server(Core::Config::WEB_SERVER_PORT);
WebSocketsServer webSocket(Core::Config::WEBSOCKET_PORT);

// Dependency Injection Container
DI::Container container;

// Controllers
std::shared_ptr<Application::RobotController> robotController;
std::shared_ptr<Infrastructure::BatteryMonitor> batteryMonitor;

// ═══════════════════════════════════════════════════════════════
// FREERTOS TASK HANDLES
// ═══════════════════════════════════════════════════════════════

TaskHandle_t gaitTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;
TaskHandle_t batteryTaskHandle = NULL;

// ═══════════════════════════════════════════════════════════════
// FREERTOS TASKS
// ═══════════════════════════════════════════════════════════════

// Task 1: GAIT CONTROL (Core 0)
// Обновление походки робота с высоким приоритетом
void gaitTask(void* parameter) {
    Core::Logger::log(Core::Logger::INFO, "🦾 Gait task started on core %d", xPortGetCoreID());
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50);  // 50ms = 20Hz
    
    while (true) {
        unsigned long currentTime = millis();
        
        // Обновляем походку
        robotController->update(currentTime);
        
        // Неблокирующая задержка
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

// Task 2: WEB SERVER (Core 1)
// Обработка WebSocket и HTTP запросов
void webTask(void* parameter) {
    Core::Logger::log(Core::Logger::INFO, "🌐 Web task started on core %d", xPortGetCoreID());
    
    while (true) {
        // Обработка WebSocket соединений
        webSocket.loop();
        
        // Обработка HTTP запросов
        server.handleClient();
        
        // Небольшая задержка
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms
    }
}

// Task 3: BATTERY MONITOR (Core 0)
// Мониторинг батареи с низким приоритетом
void batteryTask(void* parameter) {
    Core::Logger::log(Core::Logger::INFO, "🔋 Battery task started on core %d", xPortGetCoreID());
    
    while (true) {
        // Обновляем данные батареи
        batteryMonitor->update();
        Core::BatteryStatus status = batteryMonitor->getStatus();
        
        // Обновляем Safety Service
        container.getSafetyService()->setBatteryStatus(status);
        
        // Отправляем статус через WebSocket
        sendBatteryStatus(status);
        
        // Проверяем критический уровень
        if (status.isCritical) {
            Core::Logger::log(Core::Logger::ERROR, 
                "🔋 CRITICAL BATTERY: %.2fV", status.voltage);
        }
        
        // Задержка 5 секунд
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ═══════════════════════════════════════════════════════════════
// WEBSOCKET HANDLER
// ═══════════════════════════════════════════════════════════════

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Core::Logger::log(Core::Logger::INFO, "WebSocket [%u] disconnected", num);
            break;
            
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Core::Logger::log(Core::Logger::INFO, 
                "WebSocket [%u] connected from %d.%d.%d.%d",
                num, ip[0], ip[1], ip[2], ip[3]);
            
            // Отправляем текущий статус
            webSocket.sendTXT(num, robotController->getStatusJSON());
            break;
        }
            
        case WStype_TEXT: {
            String command = String((char*)payload);
            Core::Logger::log(Core::Logger::INFO, 
                "WebSocket [%u] command: %s", num, command.c_str());
            
            // Обрабатываем команду через RobotController
            robotController->handleCommand(command.c_str());
            
            // Отправляем обновленный статус
            webSocket.sendTXT(num, robotController->getStatusJSON());
            break;
        }
        
        default:
            break;
    }
}

// ═══════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════

void sendBatteryStatus(const Core::BatteryStatus& status) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
        "BATTERY:%.2f:%.1f:%d:%d",
        status.voltage,
        status.percentage,
        status.isLow ? 1 : 0,
        status.isCritical ? 1 : 0
    );
    webSocket.broadcastTXT(buffer);
}

void setupWiFi() {
    Core::Logger::log(Core::Logger::INFO, "Connecting to WiFi: %s", SSID);
    WiFi.begin(SSID, PASSWORD);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < Core::Config::WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Core::Logger::log(Core::Logger::INFO, "✅ WiFi connected! IP: %s", 
            WiFi.localIP().toString().c_str());
    } else {
        Core::Logger::log(Core::Logger::WARNING, "WiFi timeout, starting AP mode");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("Hexapod_Config", "12345678");
        Core::Logger::log(Core::Logger::INFO, "AP Mode IP: %s", 
            WiFi.softAPIP().toString().c_str());
    }
}

void setupWebServer() {
    // Главная страница
    server.on("/", []() {
        server.send_P(200, "text/html", html_page);
    });
    
    // API endpoint для статуса
    server.on("/api/status", []() {
        server.send(200, "application/json", robotController->getStatusJSON());
    });
    
    server.begin();
    Core::Logger::log(Core::Logger::INFO, "✅ Web server started on port %d", 
        Core::Config::WEB_SERVER_PORT);
    
    // WebSocket
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Core::Logger::log(Core::Logger::INFO, "✅ WebSocket started on port %d", 
        Core::Config::WEBSOCKET_PORT);
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
    // Serial для логов
    Serial.begin(115200);
    delay(1000);
    
    Core::Logger::log(Core::Logger::INFO, "");
    Core::Logger::log(Core::Logger::INFO, "╔═══════════════════════════════════════╗");
    Core::Logger::log(Core::Logger::INFO, "║  HEXAPOD ROBOT - Clean Architecture  ║");
    Core::Logger::log(Core::Logger::INFO, "║         With FreeRTOS Tasks          ║");
    Core::Logger::log(Core::Logger::INFO, "╚═══════════════════════════════════════╝");
    Core::Logger::log(Core::Logger::INFO, "");
    
    // Serial1 для servo controller
    Serial1.begin(9600, SERIAL_8N1, 4, 5);
    delay(100);
    
    // ═══════════════════════════════════════════════════════════
    // DEPENDENCY INJECTION
    // ═══════════════════════════════════════════════════════════
    
    Core::Logger::log(Core::Logger::INFO, "Initializing Dependency Injection...");
    container.initialize(Serial1);
    
    // Получаем контроллеры из DI Container
    robotController = container.getRobotController();
    batteryMonitor = container.getBatteryMonitor();
    
    // ═══════════════════════════════════════════════════════════
    // NETWORK SETUP
    // ═══════════════════════════════════════════════════════════
    
    setupWiFi();
    setupWebServer();
    
    // ═══════════════════════════════════════════════════════════
    // FREERTOS TASKS CREATION
    // ═══════════════════════════════════════════════════════════
    
    Core::Logger::log(Core::Logger::INFO, "Creating FreeRTOS tasks...");
    
    // Task 1: Gait Control (Core 0, Priority 2)
    xTaskCreatePinnedToCore(
        gaitTask,           // Task function
        "GaitTask",         // Name
        10000,              // Stack size (bytes)
        NULL,               // Parameters
        2,                  // Priority (2 = high)
        &gaitTaskHandle,    // Task handle
        0                   // Core 0
    );
    
    // Task 2: Web Server (Core 1, Priority 1)
    xTaskCreatePinnedToCore(
        webTask,
        "WebTask",
        10000,
        NULL,
        1,                  // Priority (1 = normal)
        &webTaskHandle,
        1                   // Core 1 (другое ядро!)
    );
    
    // Task 3: Battery Monitor (Core 0, Priority 0)
    xTaskCreatePinnedToCore(
        batteryTask,
        "BatteryTask",
        5000,
        NULL,
        0,                  // Priority (0 = low)
        &batteryTaskHandle,
        0                   // Core 0
    );
    
    Core::Logger::log(Core::Logger::INFO, "✅ All systems initialized!");
    Core::Logger::log(Core::Logger::INFO, "🚀 Hexapod ready for commands");
    Core::Logger::log(Core::Logger::INFO, "");
}

// ═══════════════════════════════════════════════════════════════
// LOOP (EMPTY - All work in FreeRTOS tasks)
// ═══════════════════════════════════════════════════════════════

void loop() {
    // Пустой! Вся работа выполняется в FreeRTOS tasks
    // Loop() можно использовать для диагностики
    vTaskDelay(pdMS_TO_TICKS(1000));
}

