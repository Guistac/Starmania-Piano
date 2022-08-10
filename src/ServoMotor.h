#pragma once

#include "IOSignals.h"

#include <arduino.h>

class ServoMotor{
public:

    static float pulsesPerWheelRevolution;
    static float maxVelocity_rps;
    static float minVelocity_rps;

    ServoMotor(uint8_t pulse, uint8_t dir, uint8_t enabled, uint8_t fault) : 
        pulsePin(pulse),
        directionPin(dir),
        enabledSignal(enabled, 5, true),
        faultSignal(fault, 50, true) {}

    uint8_t pulsePin = -1;
    uint8_t directionPin = -1;

    InputSignal enabledSignal;
    InputSignal faultSignal;
    void updateSignals();

    void init();
    void reset();

    void setVelocity(float vel_rps);
    double targetVelocity_rps;

    //profile generator
    double actualVelocity_rps;

    bool isEnabled();
    bool hasFault();

    bool isAtMaxVelocity();

    //interrupt service routine
    IntervalTimer timer;
    static void timedFunction(ServoMotor& servoMotor);
    bool b_pulseState;
    bool b_directionState;
    bool b_stopped = true;
    uint32_t previousUpdateTime_microseconds = 0;

};