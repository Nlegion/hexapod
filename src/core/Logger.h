#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
// LOGGER - Система логирования
// ═══════════════════════════════════════════════════════════════

namespace Core {

class Logger {
public:
    enum Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    static void log(Level level, const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        unsigned long timestamp = millis();
        unsigned long seconds = timestamp / 1000;
        unsigned long milliseconds = timestamp % 1000;

        const char* level_str = getLevelString(level);
        
        Serial.printf("[%02lu:%02lu:%02lu.%03lu] [%s] %s\n",
                      seconds / 3600,
                      (seconds % 3600) / 60,
                      seconds % 60,
                      milliseconds,
                      level_str,
                      buffer);
    }

private:
    static const char* getLevelString(Level level) {
        switch (level) {
            case DEBUG:   return "DEBUG";
            case INFO:    return "INFO";
            case WARNING: return "WARN";
            case ERROR:   return "ERROR";
            default:      return "UNKNOWN";
        }
    }
};

} // namespace Core

