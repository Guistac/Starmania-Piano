#include "RadioRemote.h"

#include "Config.h"

namespace RadioRemote{

    //======== public data ========

    //Button* modeTamponButton = new Button(MODE_TAMPOM_PIN);
    //Button* modeMecanumButton = new Button(MODE_MECANUM_PIN);

    //Button* homingTamponButton = new Button(HOMING_TAMPON_PIN);

    Button* modeAbsoluteButton = new Button(MODE_ABSOLUTE_PIN);
    Button* modeRelativeButton = new Button(MODE_RELATIVE_PIN);

    Switch* fastSpeedModeSwitch = new Switch(FAST_SPEED_MODE_PIN);
    Switch* slowSpeedModeSwitch = new Switch(SLOW_SPEED_MODE_PIN);

    Switch* emergencyStopSwitch = new Switch(EMERGENCY_STOP_PIN, true);
    //Button* rearmButton = new Button(REARM_BUTTON_PIN);

    Joystick* xAxisStick = new Joystick(NEGATIVE_X_AXIS_PIN, POSITIVE_X_AXIS_PIN);
    Joystick* yAxisStick = new Joystick(NEGATIVE_Y_AXIS_PIN, POSITIVE_Y_AXIS_PIN);
    Joystick* zAxisStick = new Joystick(NEGATIVE_Z_AXIS_PIN, POSITIVE_Z_AXIS_PIN);

    OutputSignal* modeMecanumTamponSignal = new OutputSignal(MODE_MECANUM_TAMPON_FEEDBACK_PIN);
    OutputSignal* modeAbsoluteRelativeSignal = new OutputSignal(MODE_ABSOLUTE_RELATIVE_FEEDBACK_PIN);

    bool b_armed = false;
    bool b_armingChanged = false;

    void init(){
        analogReadAveraging(32);

        emergencyStopSwitch->init();
        //rearmButton->init();

        //modeTamponButton->init();
        //modeMecanumButton->init();

        //homingTamponButton->init();

        modeAbsoluteButton->init();
        modeRelativeButton->init();

        fastSpeedModeSwitch->init();
        slowSpeedModeSwitch->init();

        xAxisStick->init();
        yAxisStick->init();
        zAxisStick->init();

        modeMecanumTamponSignal->init(false);
        modeAbsoluteRelativeSignal->init(false);

        emergencyStopSwitch->reset();
        reset();
    }


    void read(){
        emergencyStopSwitch->update();

        if(emergencyStopSwitch->isOff()){

            //rearmButton->update();

            //modeTamponButton->update();
            //modeMecanumButton->update();

            //homingTamponButton->update();

            modeAbsoluteButton->update();
            modeRelativeButton->update();

            fastSpeedModeSwitch->update();
            slowSpeedModeSwitch->update();

            xAxisStick->update();
            yAxisStick->update();
            zAxisStick->update();

        }else reset();

        #ifdef DEBUG_REMOTE
        Serial.println("---Radio Remote-----------------------------------------");
        if(emergencyStopSwitch->isOff()){
            Serial.printf("Rearm: %s %s\n", rearmButton->isPressed() ? "PRESS !" : (rearmButton->isDown() ? "pressed" : "not pressed"), rearmButton->isLongPressed() ? "(Long Pressed)" : "");
            Serial.printf("Mode Tampon: %s %s\n", modeTamponButton->isPressed() ? "PRESS !" : (modeTamponButton->isDown() ? "pressed" : "not pressed"), modeTamponButton->isLongPressed() ? "(Long Pressed)" : "");
            Serial.printf("Mode Mecanum: %s %s\n", modeMecanumButton->isPressed() ? "PRESS !" : (modeMecanumButton->isDown() ? "pressed" : "not pressed"), modeMecanumButton->isLongPressed() ? "(Long Pressed)" : "");
            Serial.printf("Homing Tampon: %s %s\n", homingTamponButton->isPressed() ? "PRESS !" : (homingTamponButton->isDown() ? "pressed" : "not pressed"), homingTamponButton->isLongPressed() ? "(Long Pressed)" : "");
            Serial.printf("Mode Absolu: %s %s\n", modeAbsoluteButton->isPressed() ? "PRESS !" : (modeAbsoluteButton->isDown() ? "pressed" : "not pressed"), modeAbsoluteButton->isLongPressed() ? "(Long Pressed)" : "");
            Serial.printf("Mode Relatif: %s %s\n", modeRelativeButton->isPressed() ? "PRESS !" : (modeRelativeButton->isDown() ? "pressed" : "not pressed"), modeRelativeButton->isLongPressed() ? "(Long Pressed)" : "");
            Serial.printf("Vitesse Rapide:  %s\n", fastSpeedModeSwitch->isFlipped() ? "FLIPPED" : (fastSpeedModeSwitch->isOn() ? "On" : "Off"));
            Serial.printf("Vitesse Lente:  %s\n", slowSpeedModeSwitch->isFlipped() ? "FLIPPED" : (slowSpeedModeSwitch->isOn() ? "On" : "Off"));
            Serial.printf("E-STOP:  %s\n", emergencyStopSwitch->isFlipped() ? "FLIPPED" : (emergencyStopSwitch->isOn() ? "On" : "Off"));
            Serial.printf("X: %.3f  %.3f\n", xAxisStick->getValue(), xAxisStick->getRawValue());
            Serial.printf("Y: %.3f  %.3f\n", yAxisStick->getValue(), yAxisStick->getRawValue());
            Serial.printf("Z: %.3f  %.3f\n", zAxisStick->getValue(), zAxisStick->getRawValue());
        }else{
            Serial.printf("Remote is not armed. Release E-Stop and press arm button.\n");
            Serial.printf("E-STOP:  %s\n", emergencyStopSwitch->isFlipped() ? "FLIPPED" : (emergencyStopSwitch->isOn() ? "On" : "Off"));
        }
        #endif
    }


    void reset(){

        //rearmButton->reset();

        //modeTamponButton->reset();
        //modeMecanumButton->reset();

        //homingTamponButton->reset();

        modeAbsoluteButton->reset();
        modeRelativeButton->reset();

        fastSpeedModeSwitch->reset();
        slowSpeedModeSwitch->reset();

        xAxisStick->reset();
        yAxisStick->reset();
        zAxisStick->reset();
    }

}

