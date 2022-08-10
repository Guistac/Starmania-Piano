#pragma once

#include <arduino.h>

#include "Config.h"

enum class ControlState{
    STOPPED,
    ENABLING,
    ENABLED
};

namespace MecanumRobot{
    void init();
    void loop();
}
