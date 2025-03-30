#ifndef CALIBRATIONMANAGER_H
#define CALIBRATIONMANAGER_H

#include <Arduino.h>

extern int SMov[32];  // Объявление глобальной переменной

class CalibrationManager {
public:
  void init();
  void load_calibration();
  bool isCalibrationLoaded();
  void resetToDefault();
  void save_calibration();
  void adjust_servo(int channel, int delta);
  void applyCalibration(int targetArray[]);  // Добавляем новую функцию

private:
  int calibrationOffsets[32] = { 0 };
  bool calibrationLoaded = false;
};

#endif  // CALIBRATIONMANAGER_H
