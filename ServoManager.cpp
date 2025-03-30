#include "ServoManager.h"
#include "config.h"

void ServoManager::init() {
    for(int i = 0; i < 32; i++) {
        positions[i] = DEFAULT_SERVO_POS;
        currentPositions[i] = DEFAULT_SERVO_POS;
    }
}

bool isMirroredLeg(int legIndex) {
  // Ноги 0-2 - левая сторона, 3-5 - правая (зеркальные)
  return (legIndex >= 3);
}

void ServoManager::set_position(int channel, int value) {
    if(channel < 0 || channel >= 32) return;
    positions[channel] = constrain(value, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
}

String ServoManager::generate_command(int speed) {
    String cmd;
    for(int i = 0; i < 32; i++) {
        if(positions[i] != currentPositions[i]) {
            cmd += "#" + String(i) + "P" + String(positions[i]);
            currentPositions[i] = positions[i];
        }
    }
    if(cmd.length() > 0) {
        cmd += "T" + String(speed) + "D0\r\n";
    }
    return cmd;
}
