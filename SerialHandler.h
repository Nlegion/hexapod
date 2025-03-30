#pragma once
#include <Arduino.h>
#include "config.h"

class SerialHandler {
private:
  unsigned long lastSendTime = 0;

public:
  void send_command(const String& cmd); // Только объявление
  bool check_response(); // Только объявление
};

extern SerialHandler serialHandler;
