#include "CalibrationManager.h"
#include <EEPROM.h>
#include "config.h"  // Убедитесь, что этот файл включен

void CalibrationManager::init() {
  calibrationLoaded = false;
  EEPROM.begin(512);
  load_calibration();  // Загружаем калибровку при инициализации
}

void CalibrationManager::applyCalibration(int targetArray[]) {
    for (int i = 0; i < 32; i++) {
        targetArray[i] = DEFAULT_SERVO_POS + calibrationOffsets[i];

        // Автоматическое зеркалирование для правой стороны
        int legIndex = i / 3;
        if (legIndex >= 3) { // Ноги 4-6
            targetArray[i] = DEFAULT_SERVO_POS * 2 - targetArray[i];
        }
    }

    // Установите начальную позицию для всех сервоприводов
    for (int i = 0; i < 32; i++) {
        SMov[i] = targetArray[i];
    }
}

void CalibrationManager::load_calibration() {
  EEPROM.get(0, calibrationOffsets);
  calibrationLoaded = true;  // Предполагаем, что калибровка успешно загружена
  Serial.println("Calibration loaded from EEPROM");
}

bool CalibrationManager::isCalibrationLoaded() {
  return calibrationLoaded;
}

void CalibrationManager::resetToDefault() {
  for (int i = 0; i < 32; i++) {
    calibrationOffsets[i] = 0;
  }
  save_calibration();
  calibrationLoaded = false;
  Serial.println("Calibration reset to default");
}

void CalibrationManager::save_calibration() {
  EEPROM.put(0, calibrationOffsets);
  EEPROM.commit();
  Serial.println("Calibration saved to EEPROM");
}



void CalibrationManager::adjust_servo(int channel, int delta) {
  if (channel >= 0 && channel < 32) {
    calibrationOffsets[channel] += delta;
    calibrationOffsets[channel] = constrain(calibrationOffsets[channel], -200, 200);
    save_calibration();  // Сохраняем калибровку после изменения
    Serial.printf("Servo %d adjusted by %d, new offset: %d\n", channel, delta, calibrationOffsets[channel]);
  } else {
    Serial.println("Invalid servo channel");
  }
}
