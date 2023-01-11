#include "ControlLogic.h"

#include "MecanumRobot.h"
#include "Config.h"

namespace ControlLogic{

    enum class SpeedMode{
        OFF,
        SNAIL,
        RABBIT
    };
    SpeedMode speedMode = SpeedMode::OFF;

    double xVelActual_mmPerSec = 0.0;
    double yVelActual_mmPerSec = 0.0;
    double rotVelActual_degPerSec = 0.0;

    double xVelTarg_mmPerSec = 0.0;
    double yVelTarg_mmPerSec = 0.0;
    double rotVelTarg_degPerSec = 0.0;

    double translVelLim_mmPerSec = 0.0;
    double rotVelLim_degPerSec = 0.0;
    double translAcc_mmPerSecSq = 0.0;
    double rotAcc_degPerSecSq = 0.0;

    //====== Rabbit & Snail Modes ======
    void setRabbitMode(){
        translAcc_mmPerSecSq = FAST_TRANSLATION_ACCELERATION_MMPS2;
        rotAcc_degPerSecSq = FAST_ROTATION_ACCELERATION_DEGPS2;
        translVelLim_mmPerSec = FAST_TRANSLATION_VELOCITY_LIMIT_MMPS;
        rotVelLim_degPerSec = FAST_ROTATION_VELOCITY_LIMIT_DEGPS;
        speedMode = SpeedMode::RABBIT;
        #ifdef DEBUG_STATECHANGES
        Serial.println("Setting Rabbit Speed");
        #endif
    }
    void setSnailMode(){
        translAcc_mmPerSecSq = SLOW_TRANSLATION_ACCELERATION_MMPS2;
        rotAcc_degPerSecSq = SLOW_ROTATION_ACCELERATION_DEGPS2;
        translVelLim_mmPerSec = SLOW_TRANSLATION_VELOCITY_LIMIT_MMPS;
        rotVelLim_degPerSec = SLOW_ROTATION_VELOCITY_LIMIT_DEGPS;
        speedMode = SpeedMode::SNAIL;
        #ifdef DEBUG_STATECHANGES
        Serial.println("Setting Snail Speed");
        #endif
    }

    void setZeroSpeedMode(){
        translAcc_mmPerSecSq = FAST_TRANSLATION_ACCELERATION_MMPS2;
        rotAcc_degPerSecSq = FAST_ROTATION_ACCELERATION_DEGPS2;
        translVelLim_mmPerSec = 0.0;
        rotVelLim_degPerSec = 0.0;
        xVelTarg_mmPerSec = 0.0;
        yVelTarg_mmPerSec = 0.0;
        rotVelTarg_degPerSec = 0.0;
        speedMode = SpeedMode::OFF;
        #ifdef DEBUG_STATECHANGES
        Serial.println("Setting Zero Speed Mode");
        #endif
    }

    bool isInRabbitMode(){ return speedMode == SpeedMode::RABBIT; }
    bool isInSnailMode(){ return speedMode == SpeedMode::SNAIL; }
    bool isInZeroSpeedMode(){ return speedMode == SpeedMode::OFF; }


    bool b_compassMode = false;
    void setRelativeMode(){ b_compassMode = false; }
    void setCompassMode(){ b_compassMode = true; }
    bool isInRelativeMode(){ return !b_compassMode; }
    bool isInCompassMode(){ return b_compassMode; }
    float currentHeading_degrees = 0.0;
    void zeroHeadingOnFront(){ currentHeading_degrees = 0.0; }
    void zeroHeadingOnBack(){ currentHeading_degrees = 180.0; }
    void zeroHeadingOnLeft(){ currentHeading_degrees = 90.0; }
    void zeroHeadingOnRight(){ currentHeading_degrees = -90.0; }


    Vector2 wheelPosition_millimeter[WHEEL_COUNT] = {
        FRONT_LEFT_WHEEL_POSITION_MM,   //front left
        FRONT_RIGHT_WHEEL_POSITION_MM,  //front right
        BACK_LEFT_WHEEL_POSITION_MM,    //back left
        BACK_RIGHT_WHEEL_POSITION_MM    //back right
    };

    Vector2 wheelFrictionVector_millimetersPerRevolution[WHEEL_COUNT] = {
        FRONT_LEFT_WHEEL_FRICTION_VECTOR_MMPREV,    //front left (goes back left when movement is positive)
        FRONT_RIGHT_WHEEL_FRICTION_VECTOR_MMPREV,   //front right (goes front left when movement is positive)
        BACK_LEFT_WHEEL_FRICTION_VECTOR_MMPREV,     //back left (goes back right when movement is positive)
        BACK_RIGHT_WHEEL_FRICTION_VECTOR_MMPREV     //back right (goes front right when movement is positive)
    };

    double wheelVelocity[4] = {
        0.0,    //front left
        0.0,    //front right
        0.0,    //back left
        0.0     //back right
    };

/*
    const int rotationCenterCount = 9;
    Vector2 rotationCenters[rotationCenterCount] = {
        {0.0, 0.0},         //center
        {-550.0, 550.0},    //front left
        {0.0, 550.0},       //front
        {550.0, 550.0},     //front right
        {550.0, 0.0},       //right
        {550.0, -550.0},    //back right
        {0.0, -550.0},      //back
        {-550.0, -550.0},   //back left
        {-550.0, 0.0}       //left
    };
    int currentRotationCenter = 0;

    void selectNextRotationCenter(){
        if(currentRotationCenter == rotationCenterCount - 1 || currentRotationCenter == 0) currentRotationCenter = 1;
        else currentRotationCenter++;
    }

    void selectPreviousRotationCenter(){
        if(currentRotationCenter == 1 || currentRotationCenter == 0) currentRotationCenter = rotationCenterCount - 1;
        else currentRotationCenter--;
    }

    void resetRotationCenter(){
        currentRotationCenter = 0;
    }
*/

    //the fixed rotation pivot point
    Vector2 centerOfRotation{.x = 0.0, .y = 0.0};



    void setXVelocityNormalized(float v_n){
        xVelTarg_mmPerSec = v_n * translVelLim_mmPerSec;
    }
    
    void setYVelocityNormalized(float v_n){
        yVelTarg_mmPerSec = v_n * translVelLim_mmPerSec;
    }

    void setRotationalVelocityNormalized(float v_n){
        rotVelTarg_degPerSec = v_n * rotVelLim_degPerSec;
    }

    uint32_t previousUpdateTime_microseconds = 0.0;

    void update(){

        float xVelocityTarget = xVelTarg_mmPerSec;
        float yVelocityTarget = yVelTarg_mmPerSec;

        //limit the translation vector magnitude to the max translation velocity
        //this effectively transforms the xy square of the left joystick into a circle
        float translationTargetVectorMagnitude = sqrt(sq(xVelocityTarget) + sq(yVelocityTarget));
        if(translationTargetVectorMagnitude != 0.0){
            xVelocityTarget /= translationTargetVectorMagnitude;
            yVelocityTarget /= translationTargetVectorMagnitude;
            if(translationTargetVectorMagnitude > translVelLim_mmPerSec) translationTargetVectorMagnitude = translVelLim_mmPerSec;
            xVelocityTarget *= translationTargetVectorMagnitude;
            yVelocityTarget *= translationTargetVectorMagnitude;
        }
        
        //limit the target velocity to the velocity limit
        if(rotVelTarg_degPerSec > rotVelLim_degPerSec) rotVelTarg_degPerSec = rotVelLim_degPerSec;
        else if(rotVelTarg_degPerSec < -rotVelLim_degPerSec) rotVelTarg_degPerSec = -rotVelLim_degPerSec;

        //get current time point and time delta from last cycle
        uint32_t now = micros();
        double deltaT_seconds = (double)(now - previousUpdateTime_microseconds) / 1000000.0;
        previousUpdateTime_microseconds = now;

        double deltaVTranslation_millimetersPerSecond = translAcc_mmPerSecSq * deltaT_seconds;

        if(xVelActual_mmPerSec < xVelocityTarget){
            xVelActual_mmPerSec += deltaVTranslation_millimetersPerSecond;
            if(xVelActual_mmPerSec > xVelocityTarget) xVelActual_mmPerSec = xVelocityTarget;
        }else if(xVelActual_mmPerSec > xVelocityTarget){
            xVelActual_mmPerSec -= deltaVTranslation_millimetersPerSecond;
            if(xVelActual_mmPerSec < xVelocityTarget) xVelActual_mmPerSec = xVelocityTarget;
        }

        if(yVelActual_mmPerSec < yVelocityTarget){
            yVelActual_mmPerSec += deltaVTranslation_millimetersPerSecond;
            if(yVelActual_mmPerSec > yVelocityTarget) yVelActual_mmPerSec = yVelocityTarget;
        }else if(yVelActual_mmPerSec > yVelocityTarget){
            yVelActual_mmPerSec -= deltaVTranslation_millimetersPerSecond;
            if(yVelActual_mmPerSec < yVelocityTarget) yVelActual_mmPerSec = yVelocityTarget;
        }

        Vector2 requestedTranslationVelocityVector{.x = xVelActual_mmPerSec, .y = yVelActual_mmPerSec};

        double deltaVRotation_degreesPerSecond = rotAcc_degPerSecSq * deltaT_seconds;
        double rotVelPrev_degreesPerSecond = rotVelActual_degPerSec;

        if(rotVelActual_degPerSec < rotVelTarg_degPerSec){
            rotVelActual_degPerSec += deltaVRotation_degreesPerSecond;
            if(rotVelActual_degPerSec > rotVelTarg_degPerSec) rotVelActual_degPerSec = rotVelTarg_degPerSec;
        }else if(rotVelActual_degPerSec > rotVelTarg_degPerSec){
            rotVelActual_degPerSec -= deltaVRotation_degreesPerSecond;
            if(rotVelActual_degPerSec < rotVelTarg_degPerSec) rotVelActual_degPerSec = rotVelTarg_degPerSec;
        }


        //get the current heading by integrating angular velocity
        double averageRotationVel = (rotVelActual_degPerSec + rotVelPrev_degreesPerSecond) / 2.0;
        double rotPosDelta = averageRotationVel * deltaT_seconds;
        currentHeading_degrees += rotPosDelta * HEADING_CORRECTION_FACTOR; //multipply by a calibrated correction factor 

        //in compass mode we use the current heading to correct the angle of the translation vector
        //this way we can keep moving in a fixed direction
        if(b_compassMode){
            
            //get info about current requested velocity
            //front is 0, right is PI/2, left is -PI/2
            //the value needs to be inverted to match the real movement
            double requestedVelocityHeading_radians = -atan2(requestedTranslationVelocityVector.x, requestedTranslationVelocityVector.y);
            double requestedVelocityMagnitude_mmPerS = sqrt(sq(requestedTranslationVelocityVector.x) + sq(requestedTranslationVelocityVector.y));

            //get the current heading in normalized radians
            double currentHeading_radians = PI * 2.0 * currentHeading_degrees / 360.0;
            while(currentHeading_radians > PI) currentHeading_radians -= 2.0 * PI;
            while(currentHeading_radians < -PI) currentHeading_radians += 2.0 * PI;

            //we need to subtract the current heading angle from the requested velocity vector to keep moving in the same direction as the heading
            //at the same time subtract 90° to match the joystick angle
            double correctedVelocityHeading_radians = requestedVelocityHeading_radians + currentHeading_radians + (PI / 2.0);
            while(correctedVelocityHeading_radians < -PI) correctedVelocityHeading_radians += 2.0 * PI;
            while(correctedVelocityHeading_radians > PI) correctedVelocityHeading_radians -= 2.0 * PI;

            //get unit vector with the correct heading and set its magnitude to equal the original requested velocity
            Vector2 requestedVelocityHeadingCorrectedVector{.x = cos(correctedVelocityHeading_radians), .y = sin(correctedVelocityHeading_radians)};
            requestedVelocityHeadingCorrectedVector.x *= requestedVelocityMagnitude_mmPerS;
            requestedVelocityHeadingCorrectedVector.y *= requestedVelocityMagnitude_mmPerS;

            //copy the corrected vector to the original vector so we can apply it later
            requestedTranslationVelocityVector = requestedVelocityHeadingCorrectedVector;
        }

        //calculate target velocity for each wheel
        for(int i = 0; i < WHEEL_COUNT; i++){
            //start calculation from zero
            wheelVelocity[i] = 0.0;

            //————————————X component velocity————————————
            wheelVelocity[i] += requestedTranslationVelocityVector.x / wheelFrictionVector_millimetersPerRevolution[i].x;

            //————————————Y component velocity————————————
            wheelVelocity[i] += requestedTranslationVelocityVector.y / wheelFrictionVector_millimetersPerRevolution[i].y;

            //———————Rotational Component velocity————————

            //wheel position relative to rotation center
            Vector2 relativeWheelPosition;
            relativeWheelPosition.x = wheelPosition_millimeter[i].x - centerOfRotation.x;
            relativeWheelPosition.y = wheelPosition_millimeter[i].y - centerOfRotation.y;

            float rotationRadius = sqrt(sq(relativeWheelPosition.x) + sq(relativeWheelPosition.y));
            float rotationCirclePerimeter = 2.0 * PI * rotationRadius;
            float rotationVectorMagnitude = rotationCirclePerimeter * rotVelActual_degPerSec / 360.0;
            rotationVectorMagnitude *= -1.0; //invert rotation vector to match remote control

            //vector perpendicular to radius of wheel position around rotation center vector
            Vector2 wheelRotationVector;
            wheelRotationVector.x = -relativeWheelPosition.y;
            wheelRotationVector.y = relativeWheelPosition.x;

            //normalize the perpendicular vector
            double normalisationMagnitude = sqrt(sq(wheelRotationVector.x) + sq(wheelRotationVector.y));
            if(normalisationMagnitude != 0.0){
                wheelRotationVector.x /= normalisationMagnitude;
                wheelRotationVector.y /= normalisationMagnitude;

                //set the rotation vector magnitude to the rotation speed
                wheelRotationVector.x *= rotationVectorMagnitude;
                wheelRotationVector.y *= rotationVectorMagnitude;
            }else{
                wheelRotationVector.x = 0.0;
                wheelRotationVector.y = 0.0;
            }

            //decompose the rotation vector and add to wheel velocity
            wheelVelocity[i] += wheelRotationVector.x / wheelFrictionVector_millimetersPerRevolution[i].x;
            wheelVelocity[i] += wheelRotationVector.y / wheelFrictionVector_millimetersPerRevolution[i].y;
        }

        #ifdef DEBUG_CONTROLLOGIC
            Serial.println("---Control Logic---------------------------");
            Serial.printf("Velocity (mm/s & deg/s)   X: %.3f  Y: %.3f  R: %.3f\n", xVelActual_mmPerSec, yVelActual_mmPerSec, rotVelActual_degPerSec);
            Serial.printf("Velocity Mode: %s\n", b_rabbitMode ? "Rabbit" : "Snail");
            Serial.printf("Translation Limit %.3fmm/s %.3fmm/s2\n", translVelLim_mmPerSec, translAcc_mmPerSecSq);
            Serial.printf("Rotation Limit %.3fdeg/s %.3fdeg/s2\n", rotVelLim_degPerSec, rotAcc_degPerSecSq);
            Serial.printf("Orientation Mode: %s\n", b_compassMode ? "Compass" : "Relative");
            Serial.printf("Heading: %.1fdeg\n", currentHeading_degrees);
            Serial.printf("Rotation Center (mm)   X: %.3f  Y: %.3f\n", centerOfRotation.x, centerOfRotation.y);
            Serial.printf("Translation Vector: %.3f %.3f\n", requestedTranslationVelocityVector.x, requestedTranslationVelocityVector.y);
            Serial.printf("Wheel Velocities (rps)    FL: %.3f  FR: %.3f\n                          BL: %.3f  BR: %.3f\n", wheelVelocity[0], wheelVelocity[1], wheelVelocity[2], wheelVelocity[3]);
        #endif
    }

    void reset(){
        xVelActual_mmPerSec = 0.0;
        yVelActual_mmPerSec = 0.0;
        rotVelActual_degPerSec = 0.0;
        xVelTarg_mmPerSec = 0.0;
        yVelTarg_mmPerSec = 0.0;
        rotVelTarg_degPerSec = 0.0;
        for(int i = 0; i < 4; i++) wheelVelocity[i] = 0.0;
    }

    bool isMoving(){
        return xVelActual_mmPerSec != 0.0 || yVelActual_mmPerSec != 0.0 || rotVelActual_degPerSec != 0.0;
    }
}
