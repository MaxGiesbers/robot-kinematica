#ifndef LOWLEVELDRIVER_H_
#define LOWLEVELDRIVER_H_

/**
 * @brief The Low level driver will receive positions from the High level interface and processes it to the AL5D protocol.
 * 
 * @file LowLevelDriver.h
 * @author Joris van Zeeland, Max Giesbers
 * @date 2018-09-25
 */

#include "Servo.h"
#include "SerialCommunication.h"
#include <string>
#include <vector>
#include <ros/ros.h>
namespace Lowlevel {
class LowLevelDriver{
public:
    /**
     * @brief Construct a new Low Level Driver object
     * 
     * @param port contains the entered usb port number from the user.
     */
    LowLevelDriver(const std::string port);

    /**
     * @brief Destroy the Low Level Driver object
     * 
     */
    virtual ~LowLevelDriver();
    /**
     * @brief Set the Position object will set the robot arm to a certain direction
     * 
     * @param positions a vector that contains all the positions for each servo from the robot arm.
     * @param time the time for exuctions in milliseconds.
     */
    void goToPosition(std::vector<int>& positions, short time);

    /**
     * @brief Sends a emergency stop to the robot arm
     * 
     */
    void emergencyStop();
    
private:
    SerialCommunication serial;
    /**
     * @brief Sends the initialize position of the robot arm (PARK) state.
     * 
     */
    void initialPosition();

};
}
#endif //LOWLEVELDRIVER_H_
