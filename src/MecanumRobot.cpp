#include "MecanumRobot.h"
#include "ServoMotor.h"
#include "RadioRemote.h"
#include "ControlLogic.h"
#include "IOSignals.h"

namespace MecanumRobot{

    ServoMotor servoMotors[WHEEL_COUNT] = {
        ServoMotor(
            FRONT_LEFT_SERVO_PULSE_PIN,
            FRONT_LEFT_SERVO_DIRECTION_PIN,
            FRONT_LEFT_SERVO_ENABLED_PIN,
            FRONT_LEFT_SERVO_FAULT_PIN),
        ServoMotor(
            FRONT_RIGHT_SERVO_PULSE_PIN,
            FRONT_RIGHT_SERVO_DIRECTION_PIN,
            FRONT_RIGHT_SERVO_ENABLED_PIN,
            FRONT_RIGHT_SERVO_FAULT_PIN),
        ServoMotor(
            BACK_LEFT_SERVO_PULSE_PIN,
            BACK_LEFT_SERVO_DIRECTION_PIN,
            BACK_LEFT_SERVO_ENABLED_PIN,
            BACK_LEFT_SERVO_FAULT_PIN),
        ServoMotor(
            BACK_RIGHT_SERVO_PULSE_PIN,
            BACK_RIGHT_SERVO_DIRECTION_PIN,
            BACK_RIGHT_SERVO_ENABLED_PIN,
            BACK_RIGHT_SERVO_FAULT_PIN)
    };

    typedef void (*isrFunction)();
    isrFunction isrs[WHEEL_COUNT] = {
        []{ ServoMotor::timedFunction(servoMotors[0]); },
        []{ ServoMotor::timedFunction(servoMotors[1]); },
        []{ ServoMotor::timedFunction(servoMotors[2]); },
        []{ ServoMotor::timedFunction(servoMotors[3]); }
    };

    //===== Control State & state transition timers =====

    ControlState controlState = ControlState::STOPPED;
    OutputSignal enableAllPowerStagesSignal(ENABLE_ALL_SERVO_POWER_STAGES_PIN);
    OutputPulseSignal resetAllFaultsSignal(RESET_ALL_SERVO_FAULTS_PIN, false, 50); //default low, 50ms pulse

    void requestEnablePowerStages(){ enableAllPowerStagesSignal.set(true); }
    void requestDisablePowerStages(){ enableAllPowerStagesSignal.set(false); }
    bool isRequestingPowerStageEnable() { return enableAllPowerStagesSignal.getState(); }

    bool b_previousHasFaults = false;
    void resetAllFaults(){ resetAllFaultsSignal.pulse(); }
    bool isResettingFaults(){ return resetAllFaultsSignal.isBusy(); }

    //===== Servo Status Query ======

    bool hasServoFault() {
        for(int i = 0; i < WHEEL_COUNT; i++) if(servoMotors[i].hasFault()) return true;
        return false;
    }

    bool areAllServosEnabled(){
        for(int i = 0; i < WHEEL_COUNT; i++) if(!servoMotors[i].isEnabled()) return false;
        return true;
    }

    void resetAllServos(){
        for(int i = 0; i < WHEEL_COUNT; i++) servoMotors[i].reset();
    }

    //===== Status Leds =====

    LED internalLed(INTERNAL_LED_PIN, false);
    LED greenLed(GREEN_LED_PIN, false);
    LED redLed(RED_LED_PIN, false);

    //===== Control Loop =====

    void onUpdate(){

        //============= READ INPUTS =============

        //read remote inputs
        RadioRemote::read();

        //read motor input signals
        for(int i = 0; i < 4; i++) servoMotors[i].updateSignals();
        bool b_hasFault = hasServoFault();
        bool b_allEnabled = areAllServosEnabled();
        bool b_servoOverloaded = false;
        for(int i = 0; i < WHEEL_COUNT; i++){
            if(servoMotors[i].isAtMaxVelocity()){
                b_servoOverloaded = true;
                break;
            }
        }

        //===== HANDLE STATE TRANSITIONS =====

        //--- Emergency Stop Halt State if Estop is down or we have a fault
        if(RadioRemote::emergencyStopSwitch->isOn() || b_hasFault) {
            #ifdef DEBUG_STATECHANGES 
            if(controlState != ControlState::STOPPED) Serial.println("State: Stopped");
            #endif
            controlState = ControlState::STOPPED;
        }

        //--- Enable Request if no Estop and no fault
        else if(controlState == ControlState::STOPPED) {
            controlState = ControlState::ENABLING;
            #ifdef DEBUG_STATECHANGES 
            Serial.println("State: Requesting Servo Enable...");
            #endif
        }

        //--- Enable Transition if we are waiting for enable and all are enabled
        else if(controlState == ControlState::ENABLING && b_allEnabled){
            controlState = ControlState::ENABLED;
            #ifdef DEBUG_STATECHANGES 
            Serial.println("State: Enabled");
            #endif
        }
        
        //--- Servo Disable Reaction if we are enabled and not all servos are enabled
        else if(controlState == ControlState::ENABLED && !b_allEnabled){
            controlState = ControlState::STOPPED;
            #ifdef DEBUG_STATECHANGES 
            Serial.println("State: Stopped (Not all servos are enabled anymore)");
            #endif
        }

        //--- Servo Fault Reaction if we are enabled and servos have faults
        else if(controlState == ControlState::ENABLED && b_hasFault){
            #ifdef DEBUG_STATECHANGES 
            Serial.println("State: Free (reaction to servo fault)");
            #endif
            controlState = ControlState::STOPPED;
        }    


        //====== HANDLE FAULT TRANSITIONS ======
      
        //allow clearing of servo faults in any state
        if(b_hasFault && RadioRemote::emergencyStopSwitch->isFlippedOff()){
            resetAllFaults();
            #ifdef DEBUG_STATECHANGES 
            Serial.println("Resetting Faults...");
            #endif
        }else if(!b_hasFault && b_previousHasFaults){
            #ifdef DEBUG_STATECHANGES 
            Serial.println("Cleared Faults !");
            #endif
        }else if(b_hasFault && !b_previousHasFaults){
            #ifdef DEBUG_STATECHANGES 
            for(int i = 0; i < WHEEL_COUNT; i++) if(servoMotors[i].hasFault()) Serial.printf("Servo Motor %i has Fault !\n", i);
            #endif
        }

        //update the pulse signal
        resetAllFaultsSignal.update();
        
        //remember previous fault state to detect transitions
        b_previousHasFaults = b_hasFault;


        //===== WRITE CONTROL STATE OUTPUTS =====

        switch(controlState){
            case ControlState::STOPPED:
                requestDisablePowerStages();
                break;
            case ControlState::ENABLING:
            case ControlState::ENABLED:
                requestEnablePowerStages();
                break;
        }



        //============== UPDATE CONTROL LOGIC ===============

        //apply speed settings to control logic

        if(RadioRemote::slowSpeedModeSwitch->isOn() && !ControlLogic::isInSnailMode()) ControlLogic::setSnailMode();
        else if(RadioRemote::fastSpeedModeSwitch->isOn() && !ControlLogic::isInRabbitMode()) ControlLogic::setRabbitMode();
        else if(RadioRemote::fastSpeedModeSwitch->isOff() && RadioRemote::slowSpeedModeSwitch->isOff() && !ControlLogic::isInZeroSpeedMode()) ControlLogic::setZeroSpeedMode();

        if(!ControlLogic::isMoving()){
            if(RadioRemote::modeRelativeButton->isPressed() && ControlLogic::isInCompassMode()) ControlLogic::setRelativeMode();
            else if(RadioRemote::modeAbsoluteButton->isPressed()) {
                if(ControlLogic::isInRelativeMode()) ControlLogic::setCompassMode();
                ControlLogic::zeroHeadingOnFront();
            }
        }

        RadioRemote::modeOutputSignal->set(ControlLogic::isInCompassMode());
        RadioRemote::speedOutputSignal->set(ControlLogic::isInRabbitMode());

        if(controlState == ControlState::ENABLED){
            ControlLogic::setXVelocityNormalized(RadioRemote::xAxisStick->getValue());
            ControlLogic::setYVelocityNormalized(RadioRemote::yAxisStick->getValue());
            ControlLogic::setRotationalVelocityNormalized(-RadioRemote::zAxisStick->getValue());
            ControlLogic::update();
            for(int i = 0; i < WHEEL_COUNT; i++) servoMotors[i].setVelocity(ControlLogic::wheelVelocity[i]);
        }else{
            ControlLogic::reset();
            ControlLogic::update();
            for(int i = 0; i < WHEEL_COUNT; i++) servoMotors[i].reset();
        }

        //============== SHOW CONTROL STATE ON LED ===============

        switch(controlState){
            case ControlState::STOPPED: //short flash every two seconds
                if(b_hasFault) {
                    if(RadioRemote::emergencyStopSwitch->isOn()){
                        redLed.blink(350, 50);
                        internalLed.blink(400, 400);
                    }else{
                        redLed.blink(200, 200);
                        internalLed.blink(400, 400);
                    }
                }
                else {
                    redLed.turnOn();
                    internalLed.turnOn();
                }
                greenLed.turnOff();
                break; 
            case ControlState::ENABLING: //constant on with short off time every half second
                redLed.blink(50, 300);
                greenLed.blink(50, 300);
                internalLed.blink(50, 150);
                break;
            case ControlState::ENABLED: //permanently on
                if(b_servoOverloaded){
                    redLed.blink(50, 50);
                    greenLed.blink(50, 50);
                    internalLed.blink(50, 50);
                }else{
                    redLed.turnOff();
                    if(ControlLogic::isMoving()) greenLed.blink(150, 150);
                    else greenLed.turnOn();
                    internalLed.blink(500, 100);
                }
                break; 
        }


        //============= DEBUG PRINT ==============

        #ifdef DEBUG_GENERAL
            Serial.printf("---Mecanum Robot Info---------------------------\n");
            switch(controlState){
                case ControlState::STOPPED:     Serial.println("State: HALTED"); break;
                case ControlState::ENABLING:    Serial.println("State: ENABLING"); break;
                case ControlState::ENABLED:     Serial.println("State: ENABLED"); break;
            }
            Serial.printf("Enable Request: %s\n", enableAllPowerStagesSignal.getState() ? "high" : "low");
            Serial.printf("Fault Reset: %s\n", resetAllFaultsSignal.getState() ? "high" : "low");
            Serial.printf("Servos: %s\n", b_hasFault ? "Fault" : (b_allEnabled ? "Enabled" : "Disabled"));
            for(int i = 0; i < WHEEL_COUNT; i++){
                Serial.printf("Servo %i Enabled: %s  Fault: %s  Overloaded: %s\n",
                i,
                servoMotors[i].isEnabled() ? "yes" : "no",
                servoMotors[i].hasFault() ? "yes" : "no",
                servoMotors[i].isAtMaxVelocity() ? "yes" : "no");
            }
        #endif

    }



    

    //===== SYSTEM STARTUP ======

    void init(){
        Serial.begin(9600);
        internalLed.init();
        redLed.init();
        greenLed.init();
        enableAllPowerStagesSignal.init(false);
        resetAllFaultsSignal.init();
        RadioRemote::init();
        for(int i = 0; i < WHEEL_COUNT; i++){
            servoMotors[i].init();
            servoMotors[i].timer.begin(isrs[i], 100);
        }
        ControlLogic::reset();
        ControlLogic::setZeroSpeedMode();
        ControlLogic::setRelativeMode();
        controlState = ControlState::STOPPED;
        requestDisablePowerStages();
    }

    //=============== MAIN RUNTIME ==============

    uint32_t lastUpdateTime_microseconds = 0;
    uint32_t updateInterval_microseconds = 1000000.0 / UPDATE_FREQUENCY;

    void loop(){
        uint32_t now_microseconds = micros();
        if(now_microseconds > lastUpdateTime_microseconds + updateInterval_microseconds){
            lastUpdateTime_microseconds = now_microseconds;
            onUpdate();
        }
    }

}
