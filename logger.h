// === logger.h ===
#pragma once
#include <Arduino.h>

class Logger {
public:
    enum Level { INFO, WARNING, ERROR };

    static void log(Level level, const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        Serial.printf("[%s] %s\n", level_str(level), buffer);
    }

private:
    static const char* level_str(Level l) {
        switch(l) {
            case INFO: return "INFO";
            case WARNING: return "WARN";
            case ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }
};
