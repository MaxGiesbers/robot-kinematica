#include "ros/ros.h"
#include "al5d_controller/LowLevel/Servo.h"

namespace Lowlevel {
Servo::Servo(const std::string aName, const signed short aMinDegrees, const signed short aMaxDegrees,
            const unsigned short aMinPW, const unsigned short aMaxPW): 
                maxDegrees(aMaxDegrees),
                minDegrees(aMinDegrees),
                maxPW(aMaxPW),
                minPW(aMinPW),
                name(aName)
{
}

Servo::Servo(const Servo& aServo):  maxDegrees(aServo.maxDegrees),
                                    minDegrees(aServo.minDegrees),
                                    maxPW(aServo.maxPW),
                                    minPW(aServo.minPW),
                                    name(aServo.name)
{
}

/*virtual*/ Servo::~Servo()
{
}

Servo& Servo::operator=(const Servo& aServo)
{
if (this != &aServo)
{
    maxDegrees = aServo.maxDegrees;
    minDegrees = aServo.minDegrees;
    maxPW = aServo.maxPW;
    minPW = aServo.minPW;
}
return *this;
}

int Servo::degreesToPW(signed short aTargetDegrees)
{
    if(aTargetDegrees >= minDegrees && aTargetDegrees <= maxDegrees)
    {
        return (aTargetDegrees - minDegrees) * (maxPW - minPW) / (maxDegrees - minDegrees) + minPW;
    } else {
        ROS_WARN("Degrees out of range, degrees must be between %d and %d", minDegrees, maxDegrees);
        return false;
    }
}
}