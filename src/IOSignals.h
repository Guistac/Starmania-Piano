#pragma once

#include <Arduino.h>

class InputSignal{
public:

    InputSignal(uint8_t _pin, uint32_t _validationDelayMillis, bool invert) : pin(_pin), validationDelay_milliseconds(_validationDelayMillis), b_invert(invert) {}

    void init(){
        pinMode(pin, INPUT_PULLUP);
        if(b_invert) b_signal = !digitalRead(pin);
        else b_signal = digitalRead(pin);
        b_reading = b_signal;
    }

    void update(){
        uint32_t now_milliseconds = millis();
        bool b_previousReading = b_reading;
        if(b_invert) b_reading = !digitalRead(pin);
        else b_reading = digitalRead(pin);
        b_updated = false;
        if(b_reading != b_previousReading) updateTime_milliseconds = now_milliseconds;
        else if(b_signal != b_reading && now_milliseconds - updateTime_milliseconds > validationDelay_milliseconds) {
            b_signal = b_reading;
            b_updated = true;
        }
    }

    bool getState(){
        return b_signal;
    }

    bool changed(){
        return b_updated;
    }

private:

    uint8_t pin;
    uint32_t validationDelay_milliseconds = 10;
    bool b_invert = false;
    bool b_signal = false;
    bool b_updated = false;

    bool b_reading = false;
    uint32_t updateTime_milliseconds = 0;
};

class OutputSignal{
    public:

    OutputSignal(uint8_t _pin) : pin(_pin){}

    void init(bool b_defaultState){
        pinMode(pin, OUTPUT);
        digitalWrite(pin, b_defaultState);
        b_state = b_defaultState;
    }

    void set(bool _state){
        digitalWrite(pin, _state);
        b_state = _state;
    }

    bool getState(){
        return b_state;
    }

    private:

    uint8_t pin;
    bool b_state;
};


class OutputPulseSignal{
public:

    OutputPulseSignal(uint8_t _pin, bool _defaultState, uint32_t _pulselengthmillis) : pin(_pin), b_defaultState(_defaultState), pulseLength_milliseconds(_pulselengthmillis) {}

    void init(){
        pinMode(pin, OUTPUT);
        digitalWrite(pin, b_defaultState);
        b_state = b_defaultState;
    }

    void update(){
        uint32_t now_millis = millis();
        if(b_doPulse && now_millis - pulseTime_milliseconds < pulseLength_milliseconds) {
            digitalWrite(pin, !b_defaultState);
            b_state = !b_defaultState;
        }
        else {
            digitalWrite(pin, b_defaultState);
            b_doPulse = false;
            b_state = b_defaultState;
        }
    }

    void pulse(){
        b_doPulse = true;
        pulseTime_milliseconds = millis();
    }

    bool getState(){
        return b_state;
    }

    bool isBusy(){
        return millis() - pulseTime_milliseconds < pulseLength_milliseconds * 2;
    }

private:

    uint8_t pin;
    bool b_doPulse = false;
    bool b_defaultState;
    bool b_state = false;
    uint32_t pulseLength_milliseconds = 50;
    uint32_t pulseTime_milliseconds = 0;

};

class LED{
public:
    
    LED(uint8_t pin_, bool startState) {
        pin = pin_;
        state = startState;
    }

    void init(){
        pinMode(pin, OUTPUT);
        digitalWrite(pin, state);
    }

    void turnOn(){
        digitalWrite(pin, true);
        state = true;
    }

    void turnOff(){
        digitalWrite(pin, false);
        state = false;
    }

    void set(bool state){
        if(state) turnOn();
        else turnOff();
    }

    void blink(uint32_t onTime, uint32_t offTime){
        set(millis() % (offTime + onTime) < onTime);
    }

private:

    bool state = false;
    uint8_t pin = -1;
};

