// spider_esp32_3_mg90_6.ino
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "page_html.h"
#include "config.h"
#include "ServoManager.h"
#include "SerialHandler.h"  // Убедитесь, что этот файл включен
#include "GaitController.h"
#include "CalibrationManager.h"
#include "SafetyMonitor.h"
#include <EEPROM.h>

bool isMoving = false;
unsigned long lastControlUpdate = 0;
String CommOut = "";
String CommIn = "";
String lastComm = "";
int StepSpeed = 100;  // Экспериментируйте в диапазоне 50-300 50;
int lastSpeed = 50;
int ClawPos = 1500;
bool systemReady = false;

extern SafetyMonitor safetyMonitor;
extern GaitController gaitController;

constexpr int DEFAULT_NEUTRAL = 1500;
constexpr int LEG_RAISE_ANGLE = 790;

const char* ssid = "Homenet_plus";
const char* password = "29pronto69";
const int CONNECTION_TIMEOUT = 20;

WebSocketsServer webSocket(81);
WebServer server(80);

ServoManager servoManager;
SerialHandler serialHandler;
CalibrationManager calibrationManager;

int SMov[32];
int SAdj[32];

bool debugMode = true;  // Переменная для включения режима отладки

void printConnectionError();
void scanNetworks();
void clearControllerCache();
void disableAutoMode();
void Send_Comm();
void wait_serial_return_ok();
void manage_power();
void enter_low_power_mode();
void handle_command(const char* command, int value);
void ClwOpn();
void ClwCls();
void test_servo(int servo_id);

void setup() {
  Serial.begin(115200);  // Измените на 115200 для монитора порта
  Serial.println("\n=== SYSTEM INIT ===");

  Serial.println("\n\n=== HEXAPOD BOOT ===");
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));

  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(9600, SERIAL_8N1, 4, 5);
  while (!Serial1)
    ;  // Ожидание инициализации UART
  Serial.println("UART1 initialized (9600 8N1)");

  EEPROM.begin(512);
  Serial.println("EEPROM initialized");

  WiFi.mode(WIFI_STA);

  Serial.println("Scanning networks...");
  int networks = WiFi.scanNetworks();
  Serial.printf("Found %d networks\n", networks);

  // Установка имени хоста
  WiFi.setHostname("Hexapod");
  // Подключение
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Статус Wi-Fi: ");
    Serial.println(WiFi.status());
    if (millis() - startTime > CONNECTION_TIMEOUT * 1000) {
      Serial.println("\nConnection failed! Possible reasons:");
      printConnectionError();
      return;  // Не перезагружаемся, продолжаем работу
    }
    delay(500);
  }

  // Вывод результатов сканирования после подключения
  networks = WiFi.scanComplete();
  Serial.println("Найдено сетей: " + String(networks));
  for (int i = 0; i < networks; i++) {
    Serial.println(WiFi.SSID(i));
  }

  Serial.println("Connected to: " + String(ssid));
  Serial.println("IP: " + WiFi.localIP().toString());

  Serial.println("Starting Web Server...");
  server.on("/", []() {
    server.send(200, "text/html", PAGE_HTML);
  });

  clearControllerCache();
  disableAutoMode();

  server.begin();
  Serial.printf("Web server started at http://%s/\n", WiFi.localIP().toString().c_str());

  Serial.println("Starting WebSocket...");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  servoManager.init();
  calibrationManager.init();
  calibrationManager.load_calibration();
  calibrationManager.applyCalibration(SAdj);  // Применяем загруженную калибровку

  // Проверка загрузки калибровки
  if (!calibrationManager.isCalibrationLoaded()) {
    Serial.println("Using default calibration");
    calibrationManager.resetToDefault();  // Добавьте эту функцию
  }

  systemReady = true;
  Serial.println("=== SYSTEM READY ===");
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());

  Pos_INT();  // Переход в начальное положение
}

void printConnectionError() {
  switch (WiFi.status()) {
    case WL_NO_SSID_AVAIL:
      Serial.println("- Network not found");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("- Wrong password");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("- Connection lost");
      break;
    case WL_DISCONNECTED:
      Serial.println("- Disconnected");
      break;
    default:
      Serial.printf("- Unknown error (%d)\n", WiFi.status());
  }
}

void scanNetworks() {
  int networks = WiFi.scanNetworks();
  Serial.println("Найдено сетей: " + String(networks));
  for (int i = 0; i < networks; i++) {
    Serial.println(WiFi.SSID(i));
  }
}

void clearControllerCache() {
  // 1. Остановка
  Serial.print("~ST\r\n");

  // 2. Сброс всех настроек (зависит от модели контроллера)
  Serial.print("#RST\r\n");  // Пример команды

  // 3. Очистка буфера
  while (Serial.available()) {
    Serial.read();
  }
}

void disableAutoMode() {
  // Пример команды для отключения автономного режима
  Serial.print("#MODE=0\r\n");  // 0 = ручное управление
  delay(100);
}

void loop() {
  webSocket.loop();
  server.handleClient();

  // Только если движение активно
  if(isMoving) {
    if(millis() - lastControlUpdate > 20) {
      gaitController.update();
      String cmd = servoManager.generate_command(StepSpeed);
      serialHandler.send_command(cmd);
      lastControlUpdate = millis();
    }
  }

  serialHandler.check_response();
  manage_power();
}

/* void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
        break;
      }
    case WStype_TEXT:
      {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, payload, length);
        const char* command = doc["cmd"];
        int value = doc["val"] | 0;
        handle_command(command, value);
        break;
      }
    default:
      break;
  }
} */

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_TEXT:
      {
        Serial.printf("Received WS message: %s\n", payload);
        DynamicJsonDocument doc(256);
        deserializeJson(doc, payload, length);
        const char* command = doc["cmd"];
        int value = doc["val"] | 0;
        Serial.printf("Command: %s, Value: %d\n", command, value);
        handle_command(command, value);
        break;
      }
  }
}


/* void Send_Comm() {
    String cmd;
    for (int i = 0; i < 32; i++) {
        int pos = SMov[i] + SAdj[i];
        pos = constrain(pos, MIN_SERVO_PULSE, MAX_SERVO_PULSE);

        // Пропускаем сервы в дефолтной позиции
        if (pos != DEFAULT_SERVO_POS) {
            cmd += "#" + String(i) + "P" + String(pos);
        }
    }

    if (cmd.length() == 0) return;

    cmd += "T" + String(map(StepSpeed, 50, 300, 9999, 100)) + "D0\r\n";
    serialHandler.send_command(cmd);
    wait_serial_return_ok();
} */

void Send_Comm() {
  String cmd;
  for(int i=0; i<32; i++) {
    int pos = constrain(SMov[i] + SAdj[i], MIN_SERVO_PULSE, MAX_SERVO_PULSE);
    cmd += "#" + String(i) + "P" + String(pos);
  }
  cmd += "T" + String(StepSpeed) + "D0\r\n";
  serialHandler.send_command(cmd);
  wait_serial_return_ok();
}


void wait_serial_return_ok() {
  unsigned long start = millis();
  String response;
  while (millis() - start < COMMAND_TIMEOUT) {
    while (Serial.available()) {
      char c = Serial.read();
      response += c;
      if (response.endsWith("OK")) return;
    }
    webSocket.loop();
    server.handleClient();
    delay(1);
  }
  Serial.println("Timeout waiting for OK");
}

void manage_power() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    // Усреднение показаний
    static float avgVoltage = 0;
    float rawVoltage = analogReadMilliVolts(A0) / 1000.0;  // Преобразование в вольты
    float batteryVoltage = rawVoltage * 2.0;               // Учитываем делитель напряжения (коэффициент 2.0)
    avgVoltage = 0.8 * avgVoltage + 0.2 * batteryVoltage;  // Фильтр

    if (debugMode) {
      Serial.print("Raw Voltage: ");
      Serial.print(rawVoltage);
      Serial.print(" V, Battery Voltage: ");
      Serial.print(batteryVoltage);
      Serial.print(" V, Average Voltage: ");
      Serial.print(avgVoltage);
      Serial.println(" V");
    }

    if (!debugMode && avgVoltage < 4.5) {
      Serial.printf("Low battery: %.2f V\n", avgVoltage);
      enter_low_power_mode();
    }

    lastCheck = millis();
  }
}

void enter_low_power_mode() {
  Serial.println("Entering deep sleep");
  esp_deep_sleep_start();
}

void handle_command(const char* command, int value) {
  if (strcmp(command, "move") == 0) {
     isMoving = true;
        switch (value) {
            case 1: gaitController.set_gait(GaitType::TRIPOD); break;
            case 2: gaitController.set_gait(GaitType::WAVE); break;
            case 3: gaitController.set_gait(GaitType::RIPPLE); break;
            case 4: gaitController.set_gait(GaitType::METACHROMATIC); break;
        }
    } else if (strcmp(command, "speed") == 0) {
    StepSpeed = constrain(value, 50, 300);
  } else if (strcmp(command, "claw") == 0) {
    if (value == 1) ClwOpn();
    else if (value == 2) ClwCls();
  } else if (strcmp(command, "adjust") == 0) {
    if (value == 1) Adj_HG();
    else if (value == 2) Adj_LW();
    else if (value == 3) Adj_HU();
    else if (value == 4) Adj_HD();
    else if (value == 5) Adj_LF();
    else if (value == 6) Adj_RG();
    else if (value == 7) Adj_TL();
    else if (value == 8) Adj_TR();
  } else if (strcmp(command, "shake") == 0) {
    Move_SHK();
  } else if (strcmp(command, "wave") == 0) {
    Move_WAV();
  } else if (strcmp(command, "stop") == 0) {
     isMoving = false;
    Move_STP();
  } else if (strcmp(command, "init") == 0) {
    Pos_INT();
  } else if (strcmp(command, "service") == 0) {
    Pos_SRV();
  } else if (strcmp(command, "test") == 0) {
    test_servo(value);  // value = номер сервопривода (0-31)
  }
}

//~~~~~~~~ claw open
void ClwOpn() {
  ClawPos = 500;  // Прямая установка безопасного минимума
  Serial.println("#9P" + String(ClawPos) + "T100D0\r\n");
  wait_serial_return_ok();
}
//~~~~~~~~~~claw close
void ClwCls() {
  ClawPos += 100;
  if (ClawPos >= 2000) ClawPos = 2000;
  Serial.println("#9P" + String(ClawPos) + "T100D0\r\n");
  wait_serial_return_ok();
}
//~~~~~~~~ Service position
void Pos_SRV() {
  lastSpeed = StepSpeed;
  StepSpeed = 50;
  SMov[29] = SAdj[29] + 1500;
  SMov[30] = SAdj[30] + 1500;
  SMov[31] = SAdj[31] + 1500;
  SMov[17] = SAdj[17] + 1500;
  SMov[18] = SAdj[18] + 1500;
  SMov[19] = SAdj[19] + 1500;
  SMov[1] = SAdj[1] + 1500;
  SMov[2] = SAdj[2] + 1500;
  SMov[3] = SAdj[3] + 1500;
  SMov[5] = SAdj[5] + 1500;
  SMov[6] = SAdj[6] + 1500;
  SMov[7] = SAdj[7] + 1500;
  SMov[13] = SAdj[13] + 1500;
  SMov[14] = SAdj[14] + 1500;
  SMov[15] = SAdj[15] + 1500;
  SMov[25] = SAdj[25] + 1500;
  SMov[26] = SAdj[26] + 1500;
  SMov[27] = SAdj[27] + 1500;
  Send_Comm();
  StepSpeed = lastSpeed;
}
//~~~~~~~~ Initial position (adjust all init servo here)
 /*
void Pos_INT() {
  if (!calibrationManager.isCalibrationLoaded()) {
    calibrationManager.load_calibration();
    calibrationManager.applyCalibration(SAdj);
  }

  // Сбрасываем все позиции с учётом калибровки
  for (int i = 0; i < 32; i++) {
    SMov[i] = DEFAULT_SERVO_POS - SAdj[i];
  }

  // Корректировка для зеркальных ног
  for (int leg = 0; leg < 6; leg++) {
    if (isMirroredLeg(leg)) {
      for (int joint = 0; joint < 3; joint++) {
        int servo = LEG_PINS[leg][joint];
        SMov[servo] = calculatePosition(leg, SMov[servo]);
      }
    }
  }

  Send_Comm();
} */

void Pos_INT() {
  if (!calibrationManager.isCalibrationLoaded()) {
    calibrationManager.load_calibration();
    calibrationManager.applyCalibration(SAdj);
  }

  // Сбрасываем все позиции БЕЗ калибровки
  for (int i = 0; i < 32; i++) {
    SMov[i] = DEFAULT_SERVO_POS; // Только базовая позиция
  }

  // Форсированная отправка команды
  String cmd;
  for(int i=0; i<32; i++) {
    cmd += "#" + String(i) + "P1500";
  }
  cmd += "T1000D0\r\n";
  serialHandler.send_command(cmd);
  wait_serial_return_ok();

  Serial.println("Reset to initial pose");
}



//~~~~~~~~ Stop motion
void Move_STP() {
  SMov[6] = SAdj[6] + 1500;
  SMov[7] = SAdj[7] + 1500;
  SMov[30] = SAdj[30] + 1500;
  SMov[31] = SAdj[31] + 1500;
  Send_Comm();
}
//~~~~~~~~ Shake hand
void Move_SHK() {
  SMov[5] = SAdj[5] + 1117;
  SMov[6] = SAdj[6] + 2218;
  SMov[7] = SAdj[7] + 1828;
  Send_Comm();
  SMov[7] = SAdj[7] + 1246;
  Send_Comm();
  SMov[7] = SAdj[7] + 1795;
  Send_Comm();
  SMov[7] = SAdj[7] + 1182;
  Send_Comm();
  SMov[7] = SAdj[7] + 1763;
  Send_Comm();
  SMov[7] = SAdj[7] + 1117;
  Send_Comm();
  SMov[5] = SAdj[5] + 1500;
  SMov[6] = SAdj[6] + 1500;
  SMov[7] = SAdj[7] + 1500;
  Send_Comm();
}
//~~~~~~~~ Waving hand
void Move_WAV() {
  SMov[5] = SAdj[5] + 1058;
  SMov[6] = SAdj[6] + 1975;
  SMov[7] = SAdj[7] + 2280;
  Send_Comm();
  SMov[5] = SAdj[5] + 1478;
  Send_Comm();
  SMov[5] = SAdj[5] + 1096;
  Send_Comm();
  SMov[5] = SAdj[5] + 1478;
  Send_Comm();
  SMov[5] = SAdj[5] + 1096;
  Send_Comm();
  SMov[5] = SAdj[5] + 1478;
  Send_Comm();
  SMov[5] = SAdj[5] + 1500;
  SMov[6] = SAdj[6] + 1500;
  SMov[7] = SAdj[7] + 1500;
  Send_Comm();
}
//~~~~~~~~ adjust body higher
void Adj_HG() {
  SAdj[6] -= 50;
  SAdj[7] += 25;
  SAdj[14] -= 50;
  SAdj[15] += 25;
  SAdj[26] -= 50;
  SAdj[27] += 25;
  SAdj[2] += 50;
  SAdj[3] -= 25;
  SAdj[18] += 50;
  SAdj[19] -= 25;
  SAdj[30] += 50;
  SAdj[31] -= 25;
  Pos_SRV();
}
//~~~~~~~~ adjust body lower
void Adj_LW() {
  SAdj[6] += 50;
  SAdj[7] -= 25;
  SAdj[14] += 50;
  SAdj[15] -= 25;
  SAdj[26] += 50;
  SAdj[27] -= 25;
  SAdj[2] -= 50;
  SAdj[3] += 25;
  SAdj[18] -= 50;
  SAdj[19] += 25;
  SAdj[30] -= 50;
  SAdj[31] += 25;
  Pos_SRV();
}
//~~~~~~~~ adjust head up
void Adj_HU() {
  SAdj[6] -= 50;
  SAdj[7] += 25;
  SAdj[26] += 50;
  SAdj[27] -= 25;
  SAdj[2] += 50;
  SAdj[3] -= 25;
  SAdj[30] -= 50;
  SAdj[31] += 25;
  Pos_SRV();
}
//~~~~~~~~ adjust head down
void Adj_HD() {
  SAdj[6] += 50;
  SAdj[7] -= 25;
  SAdj[26] -= 50;
  SAdj[27] += 25;
  SAdj[2] -= 50;
  SAdj[3] += 25;
  SAdj[30] += 50;
  SAdj[31] -= 25;
  Pos_SRV();
}
//~~~~~~~~ adjust body left
void Adj_LF() {
  SAdj[6] += 50;
  SAdj[7] -= 25;
  SAdj[14] += 50;
  SAdj[15] -= 25;
  SAdj[26] += 50;
  SAdj[27] -= 25;
  SAdj[2] += 50;
  SAdj[3] -= 25;
  SAdj[18] += 50;
  SAdj[19] -= 25;
  SAdj[30] += 50;
  SAdj[31] -= 25;
  Pos_SRV();
}
//~~~~~~~~ adjust body right
void Adj_RG() {
  SAdj[6] -= 50;
  SAdj[7] += 25;
  SAdj[14] -= 50;
  SAdj[15] += 25;
  SAdj[26] -= 50;
  SAdj[27] += 25;
  SAdj[2] -= 50;
  SAdj[3] += 25;
  SAdj[18] -= 50;
  SAdj[19] += 25;
  SAdj[30] -= 50;
  SAdj[31] += 25;
  Pos_SRV();
}
//~~~~~~~~ adjust twist left
void Adj_TL() {
  SAdj[5] -= 50;
  SAdj[13] -= 50;
  SAdj[25] -= 50;
  SAdj[1] -= 50;
  SAdj[17] -= 50;
  SAdj[29] -= 50;
  Pos_SRV();
}
//~~~~~~~~ adjust twist right
void Adj_TR() {
  SAdj[5] += 50;
  SAdj[13] += 50;
  SAdj[25] += 50;
  SAdj[1] += 50;
  SAdj[17] += 50;
  SAdj[29] += 50;
  Pos_SRV();
}

//~~~~~~~~ move forward

// Структура для хранения параметров движения
struct StepData {
  int index;
  int offset;
};

// Именованные константы для индексов
enum MotorIndices {
  MOTOR_A = 1,
  MOTOR_B = 2,
  MOTOR_C = 3,
  MOTOR_D = 5,
  MOTOR_E = 6,
  MOTOR_F = 7,
  MOTOR_G = 13,
  MOTOR_H = 14,
  MOTOR_I = 15,
  MOTOR_J = 17,
  MOTOR_K = 18,
  MOTOR_L = 19,
  MOTOR_M = 25,
  MOTOR_N = 26,
  MOTOR_O = 27,
  MOTOR_P = 29,
  MOTOR_Q = 30,
  MOTOR_R = 31
};

// Магические числа вынесены в константы
constexpr int BASE_OFFSET = 1500;
constexpr int STEP_OFFSET_1 = 1200;  // Обновленное значение
constexpr int STEP_OFFSET_2 = 1800;  // Обновленное значение
constexpr int STEP_OFFSET_3 = 1890;
constexpr int STEP_OFFSET_4 = 1815;
constexpr int STEP_OFFSET_5 = 1740;
constexpr int STEP_OFFSET_6 = 1665;
constexpr int STEP_OFFSET_7 = 1590;
constexpr int STEP_OFFSET_8 = 1515;
constexpr int STEP_OFFSET_9 = 1500;

// Массив шагов с параметрами
const StepData steps[] = {
  // Шаг 1
  { MOTOR_P, STEP_OFFSET_1 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1625 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1480 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1830 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1415 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 2
  { MOTOR_P, STEP_OFFSET_2 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1550 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1225 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1555 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1680 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1490 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 3
  { MOTOR_P, STEP_OFFSET_3 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1475 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1150 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1565 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 4
  { MOTOR_P, STEP_OFFSET_4 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1400 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1700 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1640 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 5
  { MOTOR_P, STEP_OFFSET_5 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1325 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1450 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1780 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1455 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1590 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 6
  { MOTOR_P, STEP_OFFSET_6 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1250 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1600 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1855 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1390 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 7
  { MOTOR_P, STEP_OFFSET_7 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1475 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1150 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1930 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1565 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 8
  { MOTOR_P, STEP_OFFSET_8 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1550 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1450 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1780 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1680 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1265 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 9
  { MOTOR_P, STEP_OFFSET_9 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1700 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1380 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1755 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1340 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 }
};

// Функция для выполнения движения
void Move_FWD() {
  const size_t step_size = 18;   // Количество элементов в одном шаге
  const size_t total_steps = 9;  // Общее количество шагов

  for (size_t step = 0; step < total_steps; step++) {
    for (size_t i = 0; i < step_size; i++) {
      const auto& data = steps[step * step_size + i];
      if (safetyMonitor.validate_command(data.index, SMov[data.index] + SAdj[data.index] + data.offset, StepSpeed)) {
        SMov[data.index] = SAdj[data.index] + data.offset;
      } else {
        Serial.println("Invalid command for servo " + String(data.index));
      }
    }
    Send_Comm();
  }
}

//~~~~~~~~ move backward
const StepData steps_bwd[] = {
  // Шаг 1
  { MOTOR_P, BASE_OFFSET },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1700 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1380 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1755 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1340 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 2
  { MOTOR_P, STEP_OFFSET_8 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1550 },
  { MOTOR_K, 790 },
  { MOTOR_L, 1890 },
  { MOTOR_A, 1450 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1780 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1680 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1265 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 3
  { MOTOR_P, STEP_OFFSET_7 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1400 },
  { MOTOR_K, 790 },
  { MOTOR_L, 1890 },
  { MOTOR_A, 1525 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1930 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1605 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1190 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 4
  { MOTOR_P, STEP_OFFSET_6 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1250 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1600 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1855 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1390 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 5
  { MOTOR_P, STEP_OFFSET_5 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1325 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1450 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1780 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1455 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1590 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 6
  { MOTOR_P, STEP_OFFSET_4 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1400 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1855 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1390 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 7
  { MOTOR_P, STEP_OFFSET_3 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1475 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1150 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1930 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1565 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 8
  { MOTOR_P, STEP_OFFSET_2 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1550 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1225 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1780 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1680 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1490 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 9
  { MOTOR_P, STEP_OFFSET_1 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1625 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1755 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1340 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 }
};

// Функция для выполнения движения назад
void Move_BWD() {
  const size_t step_size = 18;   // Количество элементов в одном шаге
  const size_t total_steps = 9;  // Общее количество шагов

  for (size_t step = 0; step < total_steps; step++) {
    for (size_t i = 0; i < step_size; i++) {
      const auto& data = steps_bwd[step * step_size + i];
      if (safetyMonitor.validate_command(data.index, SMov[data.index] + SAdj[data.index] + data.offset, StepSpeed)) {
        SMov[data.index] = SAdj[data.index] + data.offset;
      } else {
        Serial.println("Invalid command for servo " + String(data.index));
      }
    }
    Send_Comm();
  }
}
//~~~~~~~~ turn left
const StepData steps_lft[] = {
  // Шаг 1
  { MOTOR_P, STEP_OFFSET_1 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1625 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1755 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1340 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 2
  { MOTOR_P, STEP_OFFSET_2 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1550 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1225 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1780 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1680 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1265 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 3
  { MOTOR_P, STEP_OFFSET_3 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1475 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1150 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1930 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1565 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 4
  { MOTOR_P, STEP_OFFSET_4 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1400 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1855 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1390 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 5
  { MOTOR_P, STEP_OFFSET_5 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1325 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1450 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1780 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1455 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1590 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 6
  { MOTOR_P, STEP_OFFSET_6 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1250 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1600 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1700 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1380 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1640 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 7
  { MOTOR_P, STEP_OFFSET_7 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1400 },
  { MOTOR_K, 790 },
  { MOTOR_L, 1890 },
  { MOTOR_A, 1525 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1930 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1605 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1190 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 8
  { MOTOR_P, STEP_OFFSET_8 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1550 },
  { MOTOR_K, 790 },
  { MOTOR_L, 1890 },
  { MOTOR_A, 1450 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1780 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1680 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1265 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 9
  { MOTOR_P, STEP_OFFSET_9 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1700 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1380 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1755 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1340 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 }
};

// Функция для выполнения движения влево
void Move_LFT() {
  const size_t step_size = 18;   // Количество элементов в одном шаге
  const size_t total_steps = 9;  // Общее количество шагов

  for (size_t step = 0; step < total_steps; step++) {
    for (size_t i = 0; i < step_size; i++) {
      const auto& data = steps_lft[step * step_size + i];
      if (safetyMonitor.validate_command(data.index, SMov[data.index] + SAdj[data.index] + data.offset, StepSpeed)) {
        SMov[data.index] = SAdj[data.index] + data.offset;
      } else {
        Serial.println("Invalid command for servo " + String(data.index));
      }
    }
    Send_Comm();
  }
}
//~~~~~~~~ turn right
const StepData steps_rgt[] = {
  // Шаг 1
  { MOTOR_P, BASE_OFFSET },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1700 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1380 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1755 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1340 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 2
  { MOTOR_P, STEP_OFFSET_8 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1550 },
  { MOTOR_K, 790 },
  { MOTOR_L, 1890 },
  { MOTOR_A, 1450 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1780 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1680 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1265 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 3
  { MOTOR_P, STEP_OFFSET_7 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1400 },
  { MOTOR_K, 790 },
  { MOTOR_L, 1890 },
  { MOTOR_A, 1525 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1930 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1605 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1190 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 4
  { MOTOR_P, STEP_OFFSET_6 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1250 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1600 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1855 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1390 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 5
  { MOTOR_P, STEP_OFFSET_5 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1325 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1450 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1780 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1455 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1590 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 6
  { MOTOR_P, STEP_OFFSET_4 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1400 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, 790 },
  { MOTOR_C, 1890 },
  { MOTOR_D, 1855 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1390 },
  { MOTOR_N, 2090 },
  { MOTOR_O, 890 },

  // Шаг 7
  { MOTOR_P, STEP_OFFSET_3 },
  { MOTOR_Q, BASE_OFFSET },
  { MOTOR_R, STEP_OFFSET_1 },
  { MOTOR_J, 1475 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1150 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1930 },
  { MOTOR_E, BASE_OFFSET },
  { MOTOR_F, BASE_OFFSET },
  { MOTOR_G, 1530 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1565 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 8
  { MOTOR_P, STEP_OFFSET_2 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1550 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1225 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1780 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1680 },
  { MOTOR_H, 2090 },
  { MOTOR_I, 990 },
  { MOTOR_M, 1490 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 },

  // Шаг 9
  { MOTOR_P, STEP_OFFSET_1 },
  { MOTOR_Q, 790 },
  { MOTOR_R, 1990 },
  { MOTOR_J, 1625 },
  { MOTOR_K, BASE_OFFSET },
  { MOTOR_L, BASE_OFFSET },
  { MOTOR_A, 1300 },
  { MOTOR_B, BASE_OFFSET },
  { MOTOR_C, BASE_OFFSET },
  { MOTOR_D, 1630 },
  { MOTOR_E, 2090 },
  { MOTOR_F, 990 },
  { MOTOR_G, 1755 },
  { MOTOR_H, BASE_OFFSET },
  { MOTOR_I, BASE_OFFSET },
  { MOTOR_M, 1340 },
  { MOTOR_N, BASE_OFFSET },
  { MOTOR_O, 1390 }
};

// Функция для выполнения движения вправо
void Move_RGT() {
  const size_t step_size = 18;   // Количество элементов в одном шаге
  const size_t total_steps = 9;  // Общее количество шагов

  for (size_t step = 0; step < total_steps; step++) {
    for (size_t i = 0; i < step_size; i++) {
      const auto& data = steps_rgt[step * step_size + i];
      if (safetyMonitor.validate_command(data.index, SMov[data.index] + SAdj[data.index] + data.offset, StepSpeed)) {
        SMov[data.index] = SAdj[data.index] + data.offset;
      } else {
        Serial.println("Invalid command for servo " + String(data.index));
      }
    }
    Send_Comm();
  }
}

void test_servo(int servo_id) {
  if (servo_id < 0 || servo_id >= 32) {
    Serial.println("Invalid servo ID (0-31)");
    return;
  }

  Serial.println("#" + String(servo_id) + "P500T1000");
  delay(2000);
  Serial.println("#" + String(servo_id) + "P2400T1000");
  delay(2000);
  Serial.println("#" + String(servo_id) + "P1500T1000");
  delay(1000);
}
