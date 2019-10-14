#ifndef SERVO_H_
#define SERVO_H_

/**
 * @brief Servo class that calculates the pwm signal based on given number of degrees.
 * 
 * @file Servo.h
 * @author Joris van Zeeland, Max Giesbers
 * @date 2018-09-25
 */

#include <string>

namespace Lowlevel {
class Servo
{
public:
    /**
     * @brief Construct a new Servo object
     * 
     * @param aName contains a string with the name of the servo position. 
     * @param aMinDegrees contains signed short with lowest range in degrees of the servo
     * @param aMaxDegrees contains a signed short with the highest range in degrees of the servo.
     * @param aMinPW contains a unsigned short with minimal range in pwm of the servo.
     * @param aMaxPW contains a unsigned short with max range in pwm of the servo.
     */
    Servo(std::string aName, const signed short aMinDegrees, const signed short aMaxDegrees,
            const unsigned short aMinPW, const unsigned short aMaxPW);
    /**
     * @brief Construct a new Servo object
     * 
     * @param aServo contains a object reference to the servo.
     */

    Servo(const Servo& aServo);
    /**
     * @brief Destroy the Servo object
     * 
     */
    virtual ~Servo();
    /**
     * @brief Copy assignment operator 
     * 
     * @param aServo contains a servo object.
     * @return Servo& which contains the member variables of the aServo object.
     */

    Servo& operator=(const Servo& aServo);
    /**
     * @brief converts degrees to pwm.
     * 
     * @param aTargetDegrees the position which you want to reach in degrees.
     * @return int with the pwm value of the target degree.
     */
    int degreesToPW(signed short aTargetDegrees);

private:
    signed short maxDegrees;
    signed short minDegrees;
    unsigned short maxPW;
    unsigned short minPW;
    std::string name;
};
}

#endif // SERVO_H_