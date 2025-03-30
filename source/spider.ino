#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include "page_html.h"

String CommOut = "";
String CommIn = "";
String lastComm = "";
int StepSpeed = 50;
int lastSpeed = 50;
int SMov[32] = {1440, 1440, 1440, 0, 1440, 1440, 1440, 0, 0, 0, 0, 0, 1440, 1440, 1440, 0, 1440, 1440, 1440, 0, 0, 0, 0, 0, 1440, 1440, 1440, 0, 1440, 1440, 1440, 0};
int SAdj[32] = {0};
int StaBlink = 0;
unsigned long previousMillis = 0;
const long interval = 1000; // Интервал 1 секунда
int ClawPos = 1500;

// SSID and Password to your Wi-Fi network
const char* ssid = "Homenet_plus";
const char* password = "29pronto69";

WebSocketsServer webSocket(81);
ESP8266WebServer server(80);

/**
 * @brief Setup function to initialize the system.
 */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println(">> Setup");

  // Connect to existing Wi-Fi network
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connection failed!");
    // Действия при ошибке
    ESP.restart(); // Перезагрузка устройства при ошибке подключения
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() {
    server.send(200, "text/html", PAGE_HTML);
  });

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Initialize watchdog timer
  ESP.wdtEnable(WDTO_8S); // Enable watchdog timer with 8 seconds timeout
}

/**
 * @brief Main loop function to handle continuous tasks.
 */
void loop() {
  if (Serial.available() > 0) {
    char c[] = {(char)Serial.read()};
    webSocket.broadcastTXT(c, sizeof(c));
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    StaBlink = !StaBlink;
    digitalWrite(LED_BUILTIN, StaBlink ? HIGH : LOW);

    // Reset watchdog timer
    ESP.wdtFeed();
  }

  webSocket.loop();
  server.handleClient();

  if (CommOut == "w_0_1") Move_STP(); // Stop
  else if (CommOut == "w_1_1") Move_FWD(); // Forward
  else if (CommOut == "w_2_1") Move_BWD(); // Backward
  else if (CommOut == "w_3_1") Move_LFT(); // Turn Left
  else if (CommOut == "w_4_1") Move_RGT(); // Turn Right
  else if (CommOut == "w_5_3") {
    lastSpeed = StepSpeed;
    StepSpeed = 300;
    Move_SHK();
    StepSpeed = lastSpeed;
    CommOut = lastComm;
  }
  else if (CommOut == "w_6_3") {
    lastSpeed = StepSpeed;
    StepSpeed = 300;
    Move_WAV();
    StepSpeed = lastSpeed;
    CommOut = lastComm;
  }
  else if (CommOut == "w_15") {
    Pos_INT();
    CommOut = lastComm;
  }
  else if (CommOut == "w_12") {
    Pos_SRV();
    CommOut = lastComm;
  }
  else if (CommOut == "w_11_5") {
    Adj_LF();
    CommOut = lastComm;
  }
  else if (CommOut == "w_10_5") {
    Adj_RG();
    CommOut = lastComm;
  }
  else if (CommOut == "w_8_5") {
    Adj_HU();
    CommOut = lastComm;
  }
  else if (CommOut == "w_9_5") {
    Adj_HD();
    CommOut = lastComm;
  }
  else if (CommOut == "w_13") {
    Adj_HG();
    CommOut = lastComm;
  }
  else if (CommOut == "w_14") {
    Adj_LW();
    CommOut = lastComm;
  }
  else if (CommOut == "w_16") {
    Adj_TL();
    CommOut = lastComm;
  }
  else if (CommOut == "w_17") {
    Adj_TR();
    CommOut = lastComm;
  }
  else if (CommOut == "w_0_0") {
    StepSpeed = 300;
    CommOut = lastComm;
  }
  else if (CommOut == "w_7_1") {
    StepSpeed = 50;
    CommOut = lastComm;
  }
  else if (CommOut == "w_20") {
    ClwCls();
    CommOut = lastComm;
  }
  else if (CommOut == "w_21") {
    ClwOpn();
    CommOut = lastComm;
  }
}

/**
 * @brief WebSocket event handler.
 * @param num Client number.
 * @param type Event type.
 * @param payload Data payload.
 * @param length Length of the payload.
 */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:
      if (length > 0 && payload[length - 1] == '\0') {
        lastComm = CommOut;
        CommOut = (char*)payload; // Автоматическое преобразование
        webSocket.broadcastTXT(payload, length);
      } else {
        Serial.println("Error: Received text is not null-terminated");
      }
      break;
    case WStype_BIN:
      hexdump(payload, length);
      webSocket.sendBIN(num, payload, length);
      break;
    default:
      break;
  }
}

/**
 * @brief Sends a command to the serial port.
 */
void Send_Comm() {
  char buffer[256];
  int len = 0;

  for (int i = 1; i < 32; i++) {
    if (i >= 0 && i < 32 && SMov[i] >= 600 && SMov[i] <= 2280) {
      len += sprintf(buffer + len, "#%dP%d", i, SMov[i]);
    } else {
      Serial.printf("Error: Index %d out of bounds or value %d out of range\n", i, SMov[i]);
    }
  }

  len += sprintf(buffer + len, "T%dD0\r\n", StepSpeed);
  Serial.print(buffer);
  wait_serial_return_ok();
}

/**
 * @brief Waits for the serial port to return "OK".
 */
void wait_serial_return_ok() {
  char c[16];
  int num = 0;
  while (true) {
    while (Serial.available() > 0) {
      webSocket.loop();
      server.handleClient();
      c[num] = Serial.read();
      num++;
      if (num >= 15) num = 0;
    }
    if (num >= 2 && c[num - 2] == 'O' && c[num - 1] == 'K') break;
  }
}



//================================================================================= Servo Move =======================================================================
//~~~~~~~~ claw open
void ClwOpn(){
  ClawPos=1000;
  if (ClawPos <= 1000) ClawPos=1000;
  Serial.println("#9P" + String(ClawPos) + "T50D0");
  wait_serial_return_ok();
}
//~~~~~~~~~~claw close
void ClwCls(){
  ClawPos +=100;
  if (ClawPos >= 1600) ClawPos=1600;
  Serial.println("#9P" + String(ClawPos) + "T50D0");
  wait_serial_return_ok();
}
//~~~~~~~~ Service position
void Pos_SRV(){
  lastSpeed = StepSpeed;StepSpeed = 50;
  SMov[29]=SAdj[29]+1440;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1440;SMov[17]=SAdj[17]+1440;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1440;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1440;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1440;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1440;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1440;Send_Comm();
  StepSpeed = lastSpeed;
}
//~~~~~~~~ Initial position (adjust all init servo here)
void Pos_INT(){
  SAdj[29]=0;SAdj[30]=0;SAdj[31]=0;SAdj[17]=0;SAdj[18]=0;SAdj[19]=0;SAdj[1]=0;SAdj[2]=0;SAdj[3]=0;SAdj[5]=0;SAdj[6]=0;SAdj[7]=0;SAdj[13]=0;SAdj[14]=0;SAdj[15]=0;SAdj[25]=0;SAdj[26]=0;SAdj[27]=0;Send_Comm();
  Pos_SRV();
}
//~~~~~~~~ Stop motion
void Move_STP(){
  SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1440;Send_Comm();
}
//~~~~~~~~ Shake hand
void Move_SHK(){
  SMov[5]=SAdj[5]+1117;SMov[6]=SAdj[6]+2218;SMov[7]=SAdj[7]+1828;Send_Comm();
  SMov[7]=SAdj[7]+1246;Send_Comm();
  SMov[7]=SAdj[7]+1795;Send_Comm();
  SMov[7]=SAdj[7]+1182;Send_Comm();
  SMov[7]=SAdj[7]+1763;Send_Comm();
  SMov[7]=SAdj[7]+1117;Send_Comm();
  SMov[5]=SAdj[5]+1440;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;Send_Comm();
}
//~~~~~~~~ Waving hand
void Move_WAV(){
  SMov[5]=SAdj[5]+1058;SMov[6]=SAdj[6]+1975;SMov[7]=SAdj[7]+2280;Send_Comm();
  SMov[5]=SAdj[5]+1478;Send_Comm();
  SMov[5]=SAdj[5]+1096;Send_Comm();
  SMov[5]=SAdj[5]+1478;Send_Comm();
  SMov[5]=SAdj[5]+1096;Send_Comm();
  SMov[5]=SAdj[5]+1478;Send_Comm();
  SMov[5]=SAdj[5]+1440;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;Send_Comm();
}
//~~~~~~~~ adjust body higher
void Adj_HG(){
   SAdj[6]-=50;SAdj[7]+=25;SAdj[14]-=50;SAdj[15]+=25;SAdj[26]-=50;SAdj[27]+=25;SAdj[2]+=50;SAdj[3]-=25;SAdj[18]+=50;SAdj[19]-=25;SAdj[30]+=50;SAdj[31]-=25;
   Pos_SRV();
}
//~~~~~~~~ adjust body lower
void Adj_LW(){
  SAdj[6]+=50;SAdj[7]-=25;SAdj[14]+=50;SAdj[15]-=25;SAdj[26]+=50;SAdj[27]-=25;SAdj[2]-=50;SAdj[3]+=25;SAdj[18]-=50;SAdj[19]+=25;SAdj[30]-=50;SAdj[31]+=25;
  Pos_SRV();
}
//~~~~~~~~ adjust head up
void Adj_HU(){
   SAdj[6]-=50;SAdj[7]+=25;SAdj[26]+=50;SAdj[27]-=25;SAdj[2]+=50;SAdj[3]-=25;SAdj[30]-=50;SAdj[31]+=25;
   Pos_SRV();
}
//~~~~~~~~ adjust head down
void Adj_HD(){
   SAdj[6]+=50;SAdj[7]-=25;SAdj[26]-=50;SAdj[27]+=25;SAdj[2]-=50;SAdj[3]+=25;SAdj[30]+=50;SAdj[31]-=25;
   Pos_SRV();
}
//~~~~~~~~ adjust body left
void Adj_LF(){
  SAdj[6]+=50;SAdj[7]-=25;SAdj[14]+=50;SAdj[15]-=25;SAdj[26]+=50;SAdj[27]-=25;SAdj[2]+=50;SAdj[3]-=25;SAdj[18]+=50;SAdj[19]-=25;SAdj[30]+=50;SAdj[31]-=25;
  Pos_SRV();
}
//~~~~~~~~ adjust body right
void Adj_RG(){
  SAdj[6]-=50;SAdj[7]+=25;SAdj[14]-=50;SAdj[15]+=25;SAdj[26]-=50;SAdj[27]+=25;SAdj[2]-=50;SAdj[3]+=25;SAdj[18]-=50;SAdj[19]+=25;SAdj[30]-=50;SAdj[31]+=25;
  Pos_SRV();
}
//~~~~~~~~ adjust twist left
void Adj_TL(){
   SAdj[5]-=50;SAdj[13]-=50;SAdj[25]-=50;SAdj[1]-=50;SAdj[17]-=50;SAdj[29]-=50;
   Pos_SRV();
}
//~~~~~~~~ adjust twist right
void Adj_TR(){
   SAdj[5]+=50;SAdj[13]+=50;SAdj[25]+=50;SAdj[1]+=50;SAdj[17]+=50;SAdj[29]+=50;
   Pos_SRV();
}
//~~~~~~~~ move forward
void Move_FWD(){   
  SMov[29]=SAdj[29]+1490;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1625;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1480;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1830;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1415;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1690;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1225;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1555;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1490;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1890;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1475;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1150;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1565;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1815;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1700;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1380;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1640;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1740;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1325;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1455;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1590;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1665;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1250;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1600;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1855;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1390;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1590;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1525;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1930;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1605;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1190;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1515;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1265;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1440;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1700;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1380;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1755;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1340;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
}
//~~~~~~~~ move backward
void Move_BWD(){
  SMov[29]=SAdj[29]+1440;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1700;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1380;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1755;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1340;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1515;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1265;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1590;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1525;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1930;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1605;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1190;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1665;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1250;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1600;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1855;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1390;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1740;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1325;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1455;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1590;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1815;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1700;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1380;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1640;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1890;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1475;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1150;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1565;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1690;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1225;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1555;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1490;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1490;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1625;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1480;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1830;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1415;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
}
//~~~~~~~~ turn left
void Move_LFT(){
  SMov[29]=SAdj[29]+1490;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1625;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1755;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1340;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1690;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1225;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1265;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1890;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1475;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1150;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1930;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1605;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1190;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1815;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1855;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1390;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1740;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1325;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1455;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1590;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1665;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1250;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1600;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1700;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1380;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1640;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1590;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1525;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1565;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1515;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1555;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1490;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1440;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1700;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1380;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1480;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1830;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1415;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  }
//~~~~~~~~ turn right
void Move_RGT(){
  SMov[29]=SAdj[29]+1440;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1700;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1380;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1480;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1830;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1415;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1515;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1555;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1490;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1590;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+790;SMov[19]=SAdj[19]+1890;SMov[1]=SAdj[1]+1525;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+2090;SMov[15]=SAdj[15]+990;SMov[25]=SAdj[25]+1565;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1665;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1250;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1600;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1700;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1380;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1640;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1740;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1325;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1450;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1455;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1590;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1815;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1400;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+790;SMov[3]=SAdj[3]+1890;SMov[5]=SAdj[5]+1855;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1530;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1390;SMov[26]=SAdj[26]+2090;SMov[27]=SAdj[27]+890;Send_Comm();
  SMov[29]=SAdj[29]+1890;SMov[30]=SAdj[30]+1440;SMov[31]=SAdj[31]+1490;SMov[17]=SAdj[17]+1475;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1150;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1930;SMov[6]=SAdj[6]+1440;SMov[7]=SAdj[7]+1440;SMov[13]=SAdj[13]+1605;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1190;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1690;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1550;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1225;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1780;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1680;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1265;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
  SMov[29]=SAdj[29]+1490;SMov[30]=SAdj[30]+790;SMov[31]=SAdj[31]+1990;SMov[17]=SAdj[17]+1625;SMov[18]=SAdj[18]+1440;SMov[19]=SAdj[19]+1440;SMov[1]=SAdj[1]+1300;SMov[2]=SAdj[2]+1440;SMov[3]=SAdj[3]+1440;SMov[5]=SAdj[5]+1630;SMov[6]=SAdj[6]+2090;SMov[7]=SAdj[7]+990;SMov[13]=SAdj[13]+1755;SMov[14]=SAdj[14]+1440;SMov[15]=SAdj[15]+1440;SMov[25]=SAdj[25]+1340;SMov[26]=SAdj[26]+1440;SMov[27]=SAdj[27]+1390;Send_Comm();
}
//================================================================================= Servo Move end =======================================================================
