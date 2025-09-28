#pragma once
#include <Arduino.h>

class Logger {
public:
  enum Level { DEBUG,
               INFO,
               WARNING,
               ERROR };

  static void log(Level level, const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Получение текущего времени
    unsigned long ms = millis();
    uint16_t h = (ms / 3600000) % 24;
    uint8_t m = (ms / 60000) % 60;
    uint8_t s = (ms / 1000) % 60;
    uint16_t ms_remainder = ms % 1000;

    Serial.printf("[%02d:%02d:%02d.%03d] [%s] %s\n",
                  h, m, s, ms_remainder, level_str(level), buffer);
  }

private:
  static const char* level_str(Level l) {
    switch (l) {
      case DEBUG: return "DEBUG";
      case INFO: return "INFO";
      case WARNING: return "WARN";
      case ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }
};
