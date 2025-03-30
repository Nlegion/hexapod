// ServoManager.cpp
#include "ServoManager.h"

bool isMirroredLeg(int legIndex) {
  return legIndex >= 3;  // Ноги 4,5,6 (индексы 3,4,5) зеркальные
}

int calculatePosition(int legIndex, int baseOffset) {
  if (isMirroredLeg(legIndex)) {
    return DEFAULT_SERVO_POS * 2 - baseOffset;  // Инвертируем смещение
  }
  return baseOffset;
}
