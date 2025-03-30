#ifndef SERVOMANAGER_H
#define SERVOMANAGER_H

#include <Arduino.h>
#include "config.h"

bool isMirroredLeg(int legIndex);
int calculatePosition(int legIndex, int baseOffset);

class ServoManager {
private:
  int positions[32]; // Добавляем массив позиций
  int adjustments[32];

public:
  void init() {
    for (int i = 0; i < 32; i++) {
      positions[i] = DEFAULT_SERVO_POS;
      adjustments[i] = 0;
    }
  }

  void set_position(int channel, int value) {
    if (channel < 0 || channel >= 32) return;
    positions[channel] = constrain(value + adjustments[channel], MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  }

  String generate_command(int speed) { // Делаем метод членом класса
    String cmd;
    for (int i = 0; i < 32; i++) {
      if (positions[i] != DEFAULT_SERVO_POS) {
        cmd += "#" + String(i) + "P" + String(positions[i]);
      }
    }
    cmd += "T" + String(speed) + "D0\r\n";
    return cmd;
  }
};

extern ServoManager servoManager;

#endif
