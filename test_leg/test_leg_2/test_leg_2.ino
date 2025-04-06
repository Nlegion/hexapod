#include <Arduino.h>

enum LegID {
    LEG_FRONT_RIGHT,    // 0
    LEG_MIDDLE_RIGHT,   // 1
    LEG_REAR_RIGHT,     // 2
    LEG_REAR_LEFT,      // 3
    LEG_MIDDLE_LEFT,    // 4
    LEG_FRONT_LEFT,     // 5
    TOTAL_LEGS // Количество ног
};

enum JointID {
    COXA,   // Сустав ближе к телу
    FEMUR,  // Бедренный сустав
    TIBIA,  // Голенный сустав
    NUM_JOINTS
};

// Конфигурация сервоприводов для каждой ноги [COXA, FEMUR, TIBIA]
constexpr uint8_t LEG_SERVO_MAP[TOTAL_LEGS][NUM_JOINTS] = {
    /* LEG_FRONT_RIGHT */ {9, 10, 11},
    /* LEG_MIDDLE_RIGHT */ {5, 6, 7},
    /* LEG_REAR_RIGHT */ {1, 2, 3},
    /* LEG_REAR_LEFT */ {32, 31, 30},
    /* LEG_MIDDLE_LEFT */ {28, 27, 26},
    /* LEG_FRONT_LEFT */ {21, 22, 23}
};

const char* legNames[] = {
    "FRONT_RIGHT",
    "MIDDLE_RIGHT",
    "REAR_RIGHT",
    "REAR_LEFT",
    "MIDDLE_LEFT",
    "FRONT_LEFT"
};

const char* jointNames[] = {
    "COXA",
    "FEMUR",
    "TIBIA"
};

const int BAUDRATE = 9600;
const int NEUTRAL_POS = 1500;
const int TEST_OFFSET = 400;
const int MIN_PULSE = 500;
const int MAX_PULSE = 2500;
const unsigned long MOVE_DELAY = 5000;
const int EXECUTION_TIME = 1000;
const int DELAY_TIME = 800;

bool isMirroredLeg(LegID leg) {
    return leg >= LEG_REAR_LEFT;
}

int calculatePosition(LegID leg, int offset) {
    return isMirroredLeg(leg) 
        ? NEUTRAL_POS - offset 
        : NEUTRAL_POS + offset;
}

void sendToPosition(int servo, int position) {
    position = constrain(position, MIN_PULSE, MAX_PULSE);
    String cmd = "#" + String(servo) + 
               "P" + String(position) + 
               "T" + String(EXECUTION_TIME) + 
               "D" + String(DELAY_TIME) + 
               "\r\n";
    Serial1.print(cmd);
    Serial.println("Sent: " + cmd);
}

void testJoint(LegID leg, JointID joint) {
    int servo = LEG_SERVO_MAP[leg][joint];
    
    Serial.print("Testing ");
    Serial.print(legNames[leg]);
    Serial.print(" - ");
    Serial.println(jointNames[joint]);

    // Первое направление
    int pos1 = calculatePosition(leg, TEST_OFFSET);
    Serial.print("Moving to +offset: ");
    Serial.println(pos1);
    sendToPosition(servo, pos1);
    delay(1500);
    
    // Второе направление
    int pos2 = calculatePosition(leg, -TEST_OFFSET);
    Serial.print("Moving to -offset: ");
    Serial.println(pos2);
    sendToPosition(servo, pos2);
    delay(1500);
    
    // Возврат в нейтраль
    Serial.println("Returning to neutral");
    sendToPosition(servo, NEUTRAL_POS);
    delay(1500);
}

void testLeg(LegID leg) {
    Serial.print("\n=== START TESTING LEG ");
    Serial.print(legNames[leg]);
    Serial.println(" ===");
    
    for(int joint = COXA; joint < NUM_JOINTS; joint++) {
        testJoint(leg, static_cast<JointID>(joint));
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(BAUDRATE, SERIAL_8N1, 4, 5);
    delay(2000);

    Serial.println("Initializing servos...");
    for(int i = 0; i < 32; i++) {
        sendToPosition(i, NEUTRAL_POS);
        delay(50);
    }
    Serial.println("All servos initialized");
    delay(3000);
}

void loop() {
    static LegID currentLeg = LEG_FRONT_RIGHT;
    static unsigned long lastMove = 0;

    if(millis() - lastMove > MOVE_DELAY) {
        testLeg(currentLeg);
        currentLeg = static_cast<LegID>((static_cast<int>(currentLeg) + 1) % TOTAL_LEGS);
        lastMove = millis();
    }
}
