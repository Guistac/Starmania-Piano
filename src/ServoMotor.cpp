#include "ServoMotor.h"

#include "Config.h"

float ServoMotor::pulsesPerWheelRevolution = PULSES_PER_WHEEL_REVOLUTION;
float ServoMotor::maxVelocity_rps = MAX_MOTOR_VELOCITY_RPS;
float ServoMotor::minVelocity_rps = MIN_PULSE_FREQUENCY / ServoMotor::pulsesPerWheelRevolution;

void ServoMotor::init(){
    pinMode(pulsePin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    digitalWrite(pulsePin, LOW);
    digitalWrite(directionPin, LOW);
    enabledSignal.init();
    faultSignal.init();
    reset();
}

void ServoMotor::updateSignals(){
    enabledSignal.update();
    faultSignal.update();
}

bool ServoMotor::isEnabled(){
    return enabledSignal.getState();
}

bool ServoMotor::hasFault(){
    return faultSignal.getState();
}

void ServoMotor::reset(){
    b_pulseState = false;
    b_directionState = false;
    targetVelocity_rps = 0.0;
    actualVelocity_rps = 0.0;
}

void ServoMotor::setVelocity(float velocity_rps){
    if(velocity_rps > maxVelocity_rps) targetVelocity_rps = maxVelocity_rps;
    else if(velocity_rps < -maxVelocity_rps) targetVelocity_rps = -maxVelocity_rps;
    else if(abs(velocity_rps) < minVelocity_rps) targetVelocity_rps = 0.0;
    else targetVelocity_rps = velocity_rps;
}

bool ServoMotor::isAtMaxVelocity(){
    return abs(actualVelocity_rps) >= maxVelocity_rps;
}

void ServoMotor::timedFunction(ServoMotor& motor){

    //only update motion profile on a low pulse
    if(!motor.b_pulseState){

        //update time delta in seconds
        uint32_t now_microseconds = micros();
        double deltaT_seconds = (double)(now_microseconds - motor.previousUpdateTime_microseconds) / 1000000.0;
        motor.previousUpdateTime_microseconds = now_microseconds;

        float velocityTarget_rps = motor.targetVelocity_rps;

        //increment velocity to reach target velocity
        if(motor.actualVelocity_rps < velocityTarget_rps){
            motor.actualVelocity_rps += abs(ACCELERATION_RPS2 * deltaT_seconds);
            if(motor.actualVelocity_rps > velocityTarget_rps) motor.actualVelocity_rps = velocityTarget_rps;
        }else if(motor.actualVelocity_rps > velocityTarget_rps){
            motor.actualVelocity_rps -= abs(ACCELERATION_RPS2 * deltaT_seconds);
            if(motor.actualVelocity_rps < velocityTarget_rps) motor.actualVelocity_rps = velocityTarget_rps;
        }

        if(abs(motor.actualVelocity_rps) >= minVelocity_rps){
            motor.b_stopped = false;
            float pulseFrequency = abs(motor.actualVelocity_rps) * pulsesPerWheelRevolution;
            float pulseEdgeInterval_microseconds = 1000000.0 / (2.0 * pulseFrequency);
            motor.timer.update(pulseEdgeInterval_microseconds);
        }
        else {
            motor.b_stopped = true;
            motor.actualVelocity_rps = 0.0;
            float pulseEdgeInterval_microseconds = 1000000.0 / (2.0 * MIN_PULSE_FREQUENCY);
            motor.timer.update(pulseEdgeInterval_microseconds);
        }
    }

    if(motor.b_stopped) {
        motor.b_pulseState = false;
        motor.b_directionState = false;
    }
    else {
        motor.b_pulseState = !motor.b_pulseState;
        motor.b_directionState = motor.actualVelocity_rps > 0.0;
    }

    digitalWrite(motor.pulsePin, motor.b_pulseState);
    digitalWrite(motor.directionPin, motor.b_directionState);
}