#include <arduino.h>

#include "InputDevices.h"
#include "IOSignals.h"

namespace RadioRemote {

    void init();
    void read();
    void reset();

    extern Joystick* xAxisStick;
    extern Joystick* yAxisStick;
    extern Joystick* zAxisStick;

    extern Button* modeAbsoluteButton;
    extern Button* modeRelativeButton;

    extern Switch* fastSpeedModeSwitch;
    extern Switch* slowSpeedModeSwitch;

    extern Switch* emergencyStopSwitch;
    //extern Button* rearmButton;

    extern OutputSignal* controlModeOutputSignal;
    extern OutputSignal* controlStateOutputSignal;
}