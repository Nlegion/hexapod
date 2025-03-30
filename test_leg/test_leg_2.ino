#include <Arduino.h>

const int BAUDRATE = 9600;
const int NEUTRAL_POS = 1500;
const int TEST_OFFSET = 600;  // Базовое смещение
const unsigned long MOVE_DELAY = 5000; // Пауза между тестами

const int LEG_PINS[6][3] = {
  {1, 2, 3},   // Нога 1 (Левая передняя)
  {17, 18, 19},// Нога 2 (Левая средняя)
  {29, 30, 31},// Нога 3 (Левая задняя)
  {5, 6, 7},   // Нога 4 (Правая передняя) ← зеркальная
  {13, 14, 15},// Нога 5 (Правая средняя)  ← зеркальная
  {25, 26, 27} // Нога 6 (Правая задняя)   ← зеркальная
};

bool isMirroredLeg(int legIndex) {
  return legIndex >= 3; // Ноги 4,5,6 (индексы 3,4,5) зеркальные
}

int calculatePosition(int legIndex, int baseOffset) {
  if(isMirroredLeg(legIndex)) {
    return NEUTRAL_POS * 2 - baseOffset; // Инвертируем смещение
  }
  return baseOffset;
}

void sendToPosition(int servo, int position) {
  position = constrain(position, 500, 2500);
  String cmd = "#" + String(servo) + "P" + String(position) + "T500\r\n";
  Serial1.print(cmd);
}

void testLeg(int legIndex) {
  Serial.print("\nTesting Leg ");
  Serial.println(legIndex + 1);
  
  bool mirrored = isMirroredLeg(legIndex);
  int targetPos = calculatePosition(legIndex, TEST_OFFSET);

  Serial.print(mirrored ? "Mirrored leg. " : "Normal leg. ");
  Serial.println("Target position: " + String(targetPos));

  for(int joint = 0; joint < 3; joint++) {
    int servo = LEG_PINS[legIndex][joint];
    
    Serial.print("Moving servo ");
    Serial.print(servo);
    Serial.println(" to " + String(targetPos));
    
    sendToPosition(servo, targetPos);
    delay(1500);
    
    // Возврат в нейтральное положение
    Serial.println("Returning to neutral");
    sendToPosition(servo, NEUTRAL_POS);
    delay(1500);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUDRATE, SERIAL_8N1, 4, 5);
  delay(2000);

  // Инициализация всех в нейтральное положение
  String cmd;
  for(int i = 0; i < 32; i++) {
    cmd += "#" + String(i) + "P" + String(NEUTRAL_POS);
  }
  cmd += "T2000\r\n";
  Serial1.print(cmd);
  Serial.println("All servos initialized to neutral");
  delay(3000);
}

void loop() {
  static int currentLeg = 0;
  static unsigned long lastMove = 0;

  if(millis() - lastMove > MOVE_DELAY) {
    testLeg(currentLeg);
    currentLeg = (currentLeg + 1) % 6;
    lastMove = millis();
  }
} 
