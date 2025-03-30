#include "SerialHandler.h"

void SerialHandler::send_command(const String& cmd) {
  Serial.print("Отправка команды: ");
  Serial.println(cmd);
  Serial1.print(cmd);
  lastSendTime = millis();
}

bool SerialHandler::check_response() {
  static String buffer;
  while (Serial1.available()) {
    char c = Serial1.read();
    buffer += c;
    if (buffer.endsWith("OK")) {
      buffer = "";
      return true;
    }
    if (buffer.endsWith("ERR")) {
      Serial.println("Controller error: " + buffer);
      buffer = "";
      return false;
    }
  }
  return (millis() - lastSendTime > COMMAND_TIMEOUT);
}
