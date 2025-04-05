#pragma once
#include "config.h"
#include "safety.h"

class LegController {
public:
    void reset_pose() {
        Logger::log(Logger::INFO, "Resetting to neutral pose");
        
        for(int leg = 0; leg < NUM_LEGS; leg++) {
            for(int joint = 0; joint < 3; joint++) {
                int servo = leg * 3 + joint;
                int pulse = NEUTRAL + LEG_OFFSETS[leg][joint];
                
                pulse = constrain(pulse, MIN_PULSE + 100, MAX_PULSE - 100);
                
                if(SafetySystem::set_servo(servo, pulse)) {
                    Logger::log(Logger::INFO, "Set servo %d: %dus (leg %d joint %d)", 
                              servo, pulse, leg, joint);
                }
            }
        }
    }
};
