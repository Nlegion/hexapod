# 🏗️ Hexapod Architecture Analysis & Improvement Proposals

## 📅 Date: 2025-10-03
## 🎯 Status: Analysis Only (No Code Changes)

---

## 1️⃣ Анализ текущей архитектуры

### 📊 Текущее состояние

#### Структура файлов:
```
hexapod/
├── hexapod.ino           # 963 строки - монолитный main
├── config.h              # Конфигурация + данные
├── commands.h            # Header-only класс Commands
├── kinematics.h          # Header-only класс LegController
├── safety.h/.cpp         # Класс SafetySystem
├── logger.h              # Header-only класс Logger
└── page_html.h           # HTML как строка
```

#### Проблемы текущей архитектуры:

##### ❌ 1. Глобальное состояние
```cpp
// В hexapod.ino - все глобальные переменные!
WebServer server(80);
WebSocketsServer webSocket(81);
LegController hexapod;
bool is_moving = false;
MovementDirection movement_direction = MovementDirection::STOP;
GaitPhase current_phase = GaitPhase::PHASE1;
// ... еще 15+ глобальных переменных
```

**Проблемы:**
- Невозможно изолированно тестировать
- Сложно отследить кто меняет состояние
- Нет контроля доступа
- Tight coupling всех компонентов

##### ❌ 2. Процедурный стиль + ООП вперемешку
```cpp
// Функции верхнего уровня:
void handle_gait_cycle()
void handle_command(const char* cmd)
void perform_shake_gesture()

// Классы:
LegController hexapod;  // ООП
Commands::init_controller();  // Статический класс
SafetySystem::set_servo();  // Статический класс
```

**Проблемы:**
- Нет единого стиля
- Непонятно где какая логика
- Сложно расширять

##### ❌ 3. Нарушение Single Responsibility Principle
```cpp
// hexapod.ino делает ВСЁ:
// - WebServer setup
// - WebSocket обработка
// - Gait cycle управление
// - Gesture выполнение
// - Battery monitoring
// - Command parsing
// - WiFi management
```

##### ❌ 4. Header-only классы (проблемы компиляции)
```cpp
// commands.h, kinematics.h, logger.h - всё в заголовках
// Проблемы:
// - Медленная компиляция
// - Невозможно скрыть implementation
// - Дублирование кода при включении из разных мест
```

##### ❌ 5. Нет слоёв (все в одной куче)
```
Presentation (Web) ──┐
Application Logic ───┼── Всё в hexapod.ino!
Domain Logic ────────┤
Infrastructure ──────┘
```

---

## 2️⃣ Предложение: Clean Architecture (Layered)

### 🎯 Принципы Clean Architecture:

1. **Независимость от фреймворков**
2. **Тестируемость**
3. **Независимость от UI**
4. **Независимость от БД/Hardware**
5. **Dependency Rule** - зависимости направлены внутрь

### 📐 Предлагаемая структура слоёв:

```
┌─────────────────────────────────────────┐
│   PRESENTATION LAYER (Interface)        │  ← WebServer, WebSocket, CLI
├─────────────────────────────────────────┤
│   APPLICATION LAYER (Use Cases)         │  ← Movement, Gestures, Control
├─────────────────────────────────────────┤
│   DOMAIN LAYER (Business Logic)         │  ← Gait, Kinematics, Safety
├─────────────────────────────────────────┤
│   INFRASTRUCTURE LAYER (Hardware)       │  ← Servo, WiFi, Battery, Serial
└─────────────────────────────────────────┘
```

### 📁 Новая структура файлов:

```cpp
hexapod/
├── src/
│   ├── main.cpp                         // Только setup() и loop()
│   │
│   ├── domain/                          // 🎯 DOMAIN LAYER
│   │   ├── entities/
│   │   │   ├── Leg.h/cpp                    // Сущность "Нога"
│   │   │   ├── Body.h/cpp                   // Сущность "Тело"
│   │   │   └── GaitState.h                  // Value Objects
│   │   │
│   │   ├── services/
│   │   │   ├── IKinematicsService.h         // Интерфейс
│   │   │   ├── KinematicsService.h/cpp      // Реализация IK/FK
│   │   │   ├── IGaitService.h
│   │   │   ├── GaitService.h/cpp            // Tripod gait логика
│   │   │   ├── ISafetyService.h
│   │   │   └── SafetyService.h/cpp          // Проверки безопасности
│   │   │
│   │   └── repositories/
│   │       ├── IServoRepository.h           // Интерфейс для сервоприводов
│   │       └── IBatteryRepository.h         // Интерфейс для батареи
│   │
│   ├── application/                     // 🎯 APPLICATION LAYER (Use Cases)
│   │   ├── usecases/
│   │   │   ├── MoveForwardUseCase.h/cpp     // Use Case: движение вперёд
│   │   │   ├── TurnLeftUseCase.h/cpp        // Use Case: поворот влево
│   │   │   ├── PerformShakeUseCase.h/cpp    // Use Case: жест "пожатие"
│   │   │   ├── AdjustBodyHeightUseCase.h/cpp
│   │   │   └── MonitorBatteryUseCase.h/cpp
│   │   │
│   │   ├── dto/
│   │   │   ├── MovementCommand.h            // Data Transfer Objects
│   │   │   ├── ServoPosition.h
│   │   │   └── BatteryStatus.h
│   │   │
│   │   └── RobotController.h/cpp        // Координатор Use Cases
│   │
│   ├── infrastructure/                  // 🎯 INFRASTRUCTURE LAYER
│   │   ├── hardware/
│   │   │   ├── ServoRepository.h/cpp        // Реализация для реальных сервоприводов
│   │   │   ├── BatteryMonitor.h/cpp         // Реализация для ADC
│   │   │   └── SerialCommunicator.h/cpp     // Serial1 обёртка
│   │   │
│   │   ├── network/
│   │   │   ├── WiFiManager.h/cpp            // WiFi подключение
│   │   │   └── WebSocketManager.h/cpp       // WebSocket обработка
│   │   │
│   │   └── persistence/
│   │       └── ConfigStorage.h/cpp          // Сохранение конфигурации
│   │
│   ├── presentation/                    // 🎯 PRESENTATION LAYER
│   │   ├── web/
│   │   │   ├── WebController.h/cpp          // HTTP endpoints
│   │   │   ├── WebSocketController.h/cpp    // WebSocket handlers
│   │   │   └── WebInterface.h               // HTML генерация
│   │   │
│   │   └── cli/
│   │       └── SerialController.h/cpp       // Serial команды
│   │
│   ├── core/                            // 🎯 SHARED/CORE
│   │   ├── Logger.h/cpp                     // Логирование
│   │   ├── Config.h                         // Константы конфигурации
│   │   └── Types.h                          // Общие типы
│   │
│   └── di/                              // 🎯 DEPENDENCY INJECTION
│       └── Container.h/cpp                  // DI контейнер
│
├── tests/
│   ├── domain/                          // Тесты domain логики
│   ├── application/                     // Тесты use cases
│   ├── infrastructure/                  // Тесты hardware (с моками)
│   └── mocks/                           // Mock объекты
│
└── lib/                                 // Внешние библиотеки
```

---

## 3️⃣ Примеры кода в новой архитектуре

### 📦 Domain Layer Example

#### `domain/entities/Leg.h`
```cpp
#pragma once
#include "core/Types.h"

namespace Domain {

class Leg {
public:
    struct Position {
        float x, y, z;
    };
    
    struct Angles {
        float coxa, femur, tibia;
    };
    
    Leg(LegID id, const Position& neutralPosition);
    
    // Domain logic
    void setTargetPosition(const Position& target);
    Position getCurrentPosition() const;
    Angles getCurrentAngles() const;
    bool isInSafeRange() const;
    
private:
    LegID id_;
    Position currentPosition_;
    Angles currentAngles_;
    // No hardware dependencies!
};

} // namespace Domain
```

#### `domain/services/IGaitService.h`
```cpp
#pragma once
#include <memory>
#include "domain/entities/Leg.h"

namespace Domain {

class IGaitService {
public:
    virtual ~IGaitService() = default;
    
    virtual void startForwardGait() = 0;
    virtual void stopGait() = 0;
    virtual void updateGaitCycle(float deltaTime) = 0;
    virtual bool isMoving() const = 0;
};

} // namespace Domain
```

#### `domain/services/GaitService.cpp`
```cpp
#include "GaitService.h"

namespace Domain {

class GaitService : public IGaitService {
public:
    GaitService(
        std::shared_ptr<IKinematicsService> kinematics,
        std::shared_ptr<ISafetyService> safety
    ) : kinematics_(kinematics), safety_(safety) {}
    
    void startForwardGait() override {
        if (!safety_->canMove()) {
            throw std::runtime_error("Safety check failed");
        }
        isMoving_ = true;
        currentPhase_ = GaitPhase::PHASE1;
    }
    
    void updateGaitCycle(float deltaTime) override {
        if (!isMoving_) return;
        
        // Чистая бизнес-логика без привязки к hardware
        phaseProgress_ += deltaTime / phaseDuration_;
        
        if (phaseProgress_ >= 1.0f) {
            switchToNextPhase();
        }
        
        calculateLegPositions();
    }
    
private:
    std::shared_ptr<IKinematicsService> kinematics_;
    std::shared_ptr<ISafetyService> safety_;
    bool isMoving_ = false;
    GaitPhase currentPhase_;
    float phaseProgress_ = 0.0f;
    
    void calculateLegPositions() {
        // Domain логика расчёта позиций
        // Без вызовов servo! Только математика
    }
};

} // namespace Domain
```

---

### 📦 Application Layer Example

#### `application/usecases/MoveForwardUseCase.h`
```cpp
#pragma once
#include <memory>
#include "domain/services/IGaitService.h"
#include "domain/repositories/IServoRepository.h"

namespace Application {

class MoveForwardUseCase {
public:
    MoveForwardUseCase(
        std::shared_ptr<Domain::IGaitService> gait,
        std::shared_ptr<Domain::IServoRepository> servos
    ) : gait_(gait), servos_(servos) {}
    
    // Execute - главный метод Use Case
    void execute(float speed) {
        // 1. Validate input
        if (speed <= 0.0f || speed > 1.0f) {
            throw std::invalid_argument("Speed must be 0-1");
        }
        
        // 2. Execute domain logic
        gait_->setSpeed(speed);
        gait_->startForwardGait();
        
        // 3. Log event
        Logger::info("Started forward movement at speed %.2f", speed);
    }
    
    void stop() {
        gait_->stopGait();
        servos_->returnToNeutral();
    }
    
private:
    std::shared_ptr<Domain::IGaitService> gait_;
    std::shared_ptr<Domain::IServoRepository> servos_;
};

} // namespace Application
```

#### `application/RobotController.h`
```cpp
#pragma once
#include "usecases/MoveForwardUseCase.h"
#include "usecases/TurnLeftUseCase.h"
// ... other use cases

namespace Application {

// Координатор всех Use Cases
class RobotController {
public:
    RobotController(
        std::shared_ptr<MoveForwardUseCase> moveForward,
        std::shared_ptr<MoveBackwardUseCase> moveBackward,
        std::shared_ptr<TurnLeftUseCase> turnLeft
        // ... inject all use cases
    );
    
    // High-level API
    void handleCommand(const std::string& command);
    void update(float deltaTime);  // Вызывается из loop()
    
private:
    std::shared_ptr<MoveForwardUseCase> moveForward_;
    std::shared_ptr<MoveBackwardUseCase> moveBackward_;
    std::shared_ptr<TurnLeftUseCase> turnLeft_;
    // ...
};

} // namespace Application
```

---

### 📦 Infrastructure Layer Example

#### `infrastructure/hardware/ServoRepository.h`
```cpp
#pragma once
#include "domain/repositories/IServoRepository.h"

namespace Infrastructure {

class ServoRepository : public Domain::IServoRepository {
public:
    ServoRepository(HardwareSerial& serial) : serial_(serial) {}
    
    void setServoPosition(int channel, int pulse, int time) override {
        // Реальное взаимодействие с hardware
        char command[32];
        snprintf(command, sizeof(command), "#%dP%dT%d\r\n", 
                 channel, pulse, time);
        serial_.write(command);
    }
    
    int getServoPosition(int channel) const override {
        // Чтение позиции (если поддерживается)
        return cachedPositions_[channel];
    }
    
private:
    HardwareSerial& serial_;
    int cachedPositions_[32] = {0};
};

} // namespace Infrastructure
```

---

### 📦 Presentation Layer Example

#### `presentation/web/WebController.h`
```cpp
#pragma once
#include "application/RobotController.h"

namespace Presentation {

class WebController {
public:
    WebController(
        std::shared_ptr<Application::RobotController> robot,
        WebServer& server,
        WebSocketsServer& webSocket
    );
    
    void initialize();
    void handleWebSocketMessage(const String& message);
    
private:
    std::shared_ptr<Application::RobotController> robot_;
    WebServer& server_;
    WebSocketsServer& webSocket_;
    
    void setupRoutes();
    void onWebSocketEvent(uint8_t num, WStype_t type, 
                          uint8_t* payload, size_t length);
};

} // namespace Presentation
```

---

### 📦 Main.cpp (минимальный!)

```cpp
#include <Arduino.h>
#include "di/Container.h"
#include "application/RobotController.h"
#include "presentation/web/WebController.h"

// Dependency Injection Container
DI::Container container;

void setup() {
    Serial.begin(115200);
    
    // Setup DI Container
    container.registerServices();
    
    // Get controllers from DI
    auto robotController = container.get<Application::RobotController>();
    auto webController = container.get<Presentation::WebController>();
    
    // Initialize
    webController->initialize();
    
    Logger::info("Hexapod initialized");
}

void loop() {
    // Minimal loop - вся логика в контроллерах
    auto robotController = container.get<Application::RobotController>();
    auto webController = container.get<Presentation::WebController>();
    
    robotController->update(deltaTime);
    webController->handleClients();
    
    delay(1);  // Или vTaskDelay для FreeRTOS
}
```

---

## 4️⃣ Асинхронность на ESP32

### 🔄 Варианты асинхронности:

#### Вариант 1: FreeRTOS Tasks (РЕКОМЕНДУЕТСЯ)

ESP32 имеет встроенный FreeRTOS!

```cpp
// Вместо loop() - используем Tasks

void gaitTask(void* parameter) {
    auto gaitService = (GaitService*)parameter;
    
    while (true) {
        gaitService->updateGaitCycle(0.05f);  // 50ms
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Non-blocking delay
    }
}

void webServerTask(void* parameter) {
    auto webController = (WebController*)parameter;
    
    while (true) {
        webController->handleClients();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void batteryTask(void* parameter) {
    auto batteryMonitor = (BatteryMonitor*)parameter;
    
    while (true) {
        batteryMonitor->update();
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Каждые 5 секунд
    }
}

void setup() {
    // ... initialization ...
    
    // Create tasks на разных ядрах!
    xTaskCreatePinnedToCore(
        gaitTask,           // Function
        "GaitTask",         // Name
        10000,              // Stack size
        gaitService.get(),  // Parameter
        1,                  // Priority
        NULL,               // Task handle
        0                   // Core 0
    );
    
    xTaskCreatePinnedToCore(
        webServerTask,
        "WebServerTask",
        10000,
        webController.get(),
        1,
        NULL,
        1                   // Core 1 (другое ядро!)
    );
    
    xTaskCreatePinnedToCore(
        batteryTask,
        "BatteryTask",
        5000,
        batteryMonitor.get(),
        0,                  // Low priority
        NULL,
        0
    );
}

void loop() {
    // ПУСТОЙ! Всё работает в tasks
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
```

**Преимущества:**
- ✅ Истинная многозадачность
- ✅ Использование обоих ядер ESP32
- ✅ Priority-based scheduling
- ✅ Semaphores/Mutexes для синхронизации
- ✅ Event groups для координации

---

#### Вариант 2: AsyncWebServer + Callbacks

```cpp
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void onWebSocketEvent(AsyncWebSocket *server, 
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg, uint8_t *data, size_t len) {
    // Асинхронный обработчик
    if (type == WS_EVT_DATA) {
        String command = String((char*)data);
        
        // Неблокирующая обработка
        robotController->handleCommandAsync(command);
    }
}

void setup() {
    // Асинхронные routes
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = robotController->getStatusJSON();
        request->send(200, "application/json", json);
    });
    
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    
    server.begin();
}
```

**Преимущества:**
- ✅ Полностью неблокирующий web server
- ✅ Высокая производительность
- ✅ Callback-based архитектура

---

#### Вариант 3: Event-Driven Architecture + State Machine

```cpp
namespace Application {

class RobotStateMachine {
public:
    enum class State {
        IDLE,
        MOVING_FORWARD,
        TURNING_LEFT,
        PERFORMING_GESTURE,
        ERROR
    };
    
    enum class Event {
        START_FORWARD,
        START_TURN_LEFT,
        STOP,
        OBSTACLE_DETECTED,
        LOW_BATTERY
    };
    
    void handleEvent(Event event) {
        State newState = transition(currentState_, event);
        
        if (newState != currentState_) {
            onExit(currentState_);
            currentState_ = newState;
            onEnter(currentState_);
        }
    }
    
private:
    State currentState_ = State::IDLE;
    
    State transition(State current, Event event) {
        // State transition logic
        // Полностью event-driven!
    }
};

} // namespace Application
```

---

## 5️⃣ Dependency Injection Container

```cpp
// di/Container.h
#pragma once
#include <memory>
#include <map>
#include <functional>

class Container {
public:
    template<typename T>
    void registerSingleton(std::function<std::shared_ptr<T>()> factory) {
        singletons_[typeid(T).name()] = [this, factory]() {
            return std::static_pointer_cast<void>(factory());
        };
    }
    
    template<typename T>
    std::shared_ptr<T> get() {
        auto it = instances_.find(typeid(T).name());
        if (it != instances_.end()) {
            return std::static_pointer_cast<T>(it->second);
        }
        
        auto factory = singletons_[typeid(T).name()];
        auto instance = factory();
        instances_[typeid(T).name()] = instance;
        return std::static_pointer_cast<T>(instance);
    }
    
private:
    std::map<const char*, std::function<std::shared_ptr<void>()>> singletons_;
    std::map<const char*, std::shared_ptr<void>> instances_;
};

// Использование:
void Container::registerServices() {
    // Infrastructure
    registerSingleton<IServoRepository>([this]() {
        return std::make_shared<ServoRepository>(Serial1);
    });
    
    // Domain Services
    registerSingleton<IGaitService>([this]() {
        return std::make_shared<GaitService>(
            get<IKinematicsService>(),
            get<ISafetyService>()
        );
    });
    
    // Use Cases
    registerSingleton<MoveForwardUseCase>([this]() {
        return std::make_shared<MoveForwardUseCase>(
            get<IGaitService>(),
            get<IServoRepository>()
        );
    });
    
    // Controllers
    registerSingleton<RobotController>([this]() {
        return std::make_shared<RobotController>(
            get<MoveForwardUseCase>(),
            get<TurnLeftUseCase>()
            // ... all use cases
        );
    });
}
```

---

## 6️⃣ Преимущества новой архитектуры

### ✅ Тестируемость

**Было:**
```cpp
// Невозможно протестировать без hardware!
void handle_gait_cycle() {
    SafetySystem::set_servo(servo, pulse);  // Hardware call!
}
```

**Стало:**
```cpp
// Легко тестировать с mock объектами
class MockServoRepository : public IServoRepository {
    void setServoPosition(int ch, int pulse, int time) override {
        recorded_calls.push_back({ch, pulse, time});
    }
};

TEST(GaitService, "calculates correct positions") {
    auto mockServos = std::make_shared<MockServoRepository>();
    auto gait = std::make_shared<GaitService>(mockServos);
    
    gait->startForwardGait();
    gait->updateGaitCycle(0.1f);
    
    ASSERT_EQ(mockServos->recorded_calls.size(), 18);  // 6 legs * 3 joints
}
```

### ✅ Расширяемость

Добавить новый жест:
1. Создать `PerformWaveUseCase.cpp`
2. Зарегистрировать в DI Container
3. Готово! Не трогаем существующий код.

### ✅ Поддерживаемость

Каждый файл < 200 строк, одна ответственность.

### ✅ Независимость от hardware

Domain логику можно запустить на ПК для отладки!

---

## 7️⃣ План миграции (если решим внедрить)

### Фаза 1: Подготовка (1-2 недели)
1. Создать структуру папок
2. Выделить интерфейсы (IServoRepository, IGaitService)
3. Настроить DI Container

### Фаза 2: Domain Layer (2-3 недели)
1. Переместить Kinematics в Domain
2. Создать сущности (Leg, Body)
3. Реализовать GaitService
4. Покрыть тестами (100%)

### Фаза 3: Application Layer (1-2 недели)
1. Создать Use Cases
2. Создать RobotController
3. Тесты Use Cases

### Фаза 4: Infrastructure (1 неделя)
1. Реализовать ServoRepository
2. Реализовать BatteryMonitor
3. WiFiManager

### Фаза 5: Presentation (1 неделя)
1. WebController
2. WebSocketController
3. Интеграция

### Фаза 6: FreeRTOS Integration (1 неделя)
1. Создать tasks
2. Настроить priorities
3. Тестирование многозадачности

**Итого: 7-10 недель** для полной миграции.

---

## 8️⃣ Альтернативные варианты (менее радикальные)

### Вариант A: Постепенная рефакторинг текущего кода

1. Вынести глобальные переменные в классы
2. Создать RobotState класс
3. Создать GaitController класс
4. Использовать FreeRTOS для асинхронности

**Плюсы:** Быстрее, меньше изменений
**Минусы:** Не решает архитектурных проблем полностью

### Вариант B: Hybrid Architecture

Оставить текущую структуру для простых вещей, но:
- Domain Services для сложной логики (Gait, Kinematics)
- FreeRTOS tasks для параллелизма
- Интерфейсы только для hardware

**Плюсы:** Баланс между простотой и качеством
**Минусы:** Может стать "ни рыба ни мясо"

---

## 9️⃣ Рекомендации

### Для вашего проекта я рекомендую:

#### ✅ Сейчас (без больших изменений):

1. **Добавить FreeRTOS tasks** - это не требует переписывания кода!
   ```cpp
   // Минимальные изменения:
   xTaskCreate(gaitTask, "Gait", 10000, NULL, 1, NULL);
   xTaskCreate(webTask, "Web", 10000, NULL, 1, NULL);
   ```

2. **Вынести глобальные переменные в RobotState класс**
   ```cpp
   class RobotState {
   public:
       bool isMoving = false;
       MovementDirection direction = STOP;
       GaitPhase phase = PHASE1;
       // ... все глобальные переменные
   };
   
   RobotState robotState;  // Одна глобальная вместо 15+
   ```

3. **Использовать AsyncWebServer** вместо WebServer
   - Простая замена библиотеки
   - Огромный прирост производительности

#### ✅ В будущем (если проект растёт):

4. **Clean Architecture** - полная миграция
   - Если планируете масштабировать
   - Если нужна высокая тестируемость
   - Если будет команда разработчиков

---

## 🎯 Выводы

### Текущая архитектура:
- ✅ Работает
- ✅ Проста для понимания новичками
- ❌ Сложно тестировать
- ❌ Сложно масштабировать
- ❌ Tight coupling

### Clean Architecture:
- ✅ Высокая тестируемость
- ✅ Легко расширять
- ✅ Независимость от hardware
- ✅ Professional quality
- ❌ Сложнее в начале
- ❌ Требует времени на миграцию

### FreeRTOS + AsyncWebServer:
- ✅ Можно добавить прямо сейчас
- ✅ Минимальные изменения кода
- ✅ Большой прирост производительности
- ✅ Истинная многозадачность

---

## 📞 Следующие шаги

**Если хотите улучшить архитектуру:**

1. Обсудим приоритеты (производительность vs архитектура)
2. Выберем подход (радикальный vs постепенный)
3. Создадим план реализации
4. Начнём с малого (FreeRTOS tasks)

**Вопросы для обсуждения:**
- Планируется ли расширение функционала?
- Будет ли команда разработчиков?
- Какие приоритеты: скорость разработки vs качество кода?
- Сколько времени готовы вложить в рефакторинг?

---

*Документ создан: 2025-10-03*  
*Статус: Анализ без изменений кода*  
*Следующий шаг: Обсуждение подхода*

