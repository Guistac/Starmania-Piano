#pragma once

#include <arduino.h>

//=========== BUTTON ============

class Button{
public:

    Button(int p) : pin(p) {}
    
    void init(){ pinMode(pin, INPUT_PULLDOWN); }

    void update(){
        bool b_previousReading = b_reading;
        b_reading = digitalRead(pin);

        uint32_t now_milliseconds = millis();

        if(b_reading && !b_previousReading){
            pressTime_milliseconds = now_milliseconds;
        }else if(b_reading && b_previousReading){
            //regular press detection
            b_down = false;
            b_pressed = false;
            if(now_milliseconds - pressTime_milliseconds > pressValidationDelay_milliseconds){
                b_down = true;
                if(!b_wasPressed){
                    b_pressed = true;
                    b_wasPressed = true;
                }
            }
            //long press detection
            if(now_milliseconds - pressTime_milliseconds > longPressValidationDelay_milliseconds){
                b_longPressed = true;
            }
        }else{
            b_pressed = false;
            b_longPressed = false;
            b_wasPressed = false;
            b_down = false;
        }
    }
    void reset(){
        b_reading = false;
        b_pressed = false;
        b_wasPressed = false;
        b_down = false;
        b_longPressed = false;
        pressTime_milliseconds = 0;
    }

    bool isPressed(){ return b_pressed; }
    bool isDown(){ return b_down; }
    bool isLongPressed(){ return b_longPressed; }

private:

    int pin = -1;
    bool b_reading = false;
    bool b_pressed = false;
    bool b_wasPressed = false;
    bool b_down = false;
    bool b_longPressed = false;
    uint32_t pressTime_milliseconds = 0;
    const uint32_t pressValidationDelay_milliseconds = 50;
    const uint32_t longPressValidationDelay_milliseconds = 500;
};














//=========== SWITCH ============

class Switch{
public:

    Switch(int p, bool inverted = false) : pin(p), b_inverted(inverted) {}
    
    void init(){ pinMode(pin, INPUT_PULLDOWN); }

    void update(){
        bool b_previousReading = b_reading;
        if(b_inverted) b_reading = !digitalRead(pin);
        else b_reading = digitalRead(pin);

        uint32_t now_milliseconds = millis();

        if(b_previousReading != b_reading){
            b_flipState = b_reading;
            flipTime_milliseconds = now_milliseconds;
            b_flipped = false;
        }
        else if(now_milliseconds - flipTime_milliseconds > flipValidationDelay_milliseconds){
            //if output has not changed since timer start
            if(b_flipState == b_reading){
                b_flipped = false;
                if(b_isOn != b_reading) b_flipped = true;
                b_isOn = b_reading;
            }
        }
    }

    void reset(){
        b_reading = false;
        b_isOn = false;
        b_flipState = false;
        b_flipped = false;
        flipTime_milliseconds = 0;
    }

    bool isOn(){ return b_isOn; }
    bool isOff(){ return !b_isOn; }
    bool isFlipped(){ return b_flipped; }
    bool isFlippedOn(){ return b_flipped && b_isOn; }
    bool isFlippedOff(){ return b_flipped && !b_isOn; }

private:

    int pin = -1;
    bool b_inverted = false;
    bool b_reading = false;
    bool b_isOn = false;
    bool b_flipState = false;
    bool b_flipped = false;
    uint32_t flipTime_milliseconds = 0;
    const uint32_t flipValidationDelay_milliseconds = 50;

};














//=========== JOYSTICK ============

class Joystick{
public:

    Joystick(int nPin, int pPin) : negativePin(nPin), positivePin(pPin) {}
    
    void init(){
        pinMode(negativePin, INPUT);
        pinMode(positivePin, INPUT);
    }

    void update(){
        rawValue = ((float)analogRead(positivePin) / 1024.0) - (analogRead(negativePin) / 1024.0);
        smoothedValue = filter * smoothedValue + (1.0f - filter) * rawValue;
        if(abs(smoothedValue) < threshold) outputValue = 0.0;
        else {
            outputValue = map(abs(smoothedValue), threshold, 1.0, 0.0, 1.0);
            if(smoothedValue < 0.0) outputValue *= -1.0;
        }
    }


    float getRawValue(){ return rawValue; }

    float getValue(){ return outputValue; }

    void reset(){
        rawValue = 0.0;
        smoothedValue = 0.0;
        outputValue = 0.0;
    }

    int negativePin = -1;
    int positivePin = -1;

private:

    const float filter = 0.90;
    const float threshold = 0.03;

    float rawValue = 0.0;
    float smoothedValue = 0.0;
    float outputValue = 0.0;

};








