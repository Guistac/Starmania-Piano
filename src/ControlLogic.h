#include <Arduino.h>

#include "MecanumRobot.h"

struct Vector2{
    double x = 0.0;
    double y = 0.0;
};

namespace ControlLogic{

    extern double wheelVelocity[4];

    void setRabbitMode();
    void setSnailMode();
    bool isInRabbitMode();
    bool isInSnailMode();

    void setRelativeMode();
    void setCompassMode();
    bool isInRelativeMode();
    bool isInCompassMode();
    void zeroHeadingOnFront(); 
    void zeroHeadingOnBack(); 
    void zeroHeadingOnLeft();
    void zeroHeadingOnRight();

    void setXVelocityNormalized(float v);
    void setYVelocityNormalized(float v);
    void setRotationalVelocityNormalized(float v);

    void selectNextRotationCenter();
    void selectPreviousRotationCenter();
    void resetRotationCenter();

    void update();
    void reset();
    bool isStopped();

    bool isMoving();
}