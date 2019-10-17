#ifndef LOWLEVELDRIVER_H_
#define LOWLEVELDRIVER_H_

#include "Servo.h"
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/asio.hpp>

class LowLevelDriver{
public:

    LowLevelDriver(const std::string port);
    virtual ~LowLevelDriver();

    void goToPosition(std::vector<int>& positions, short time);
    void emergencyStop();
    void writeString(std::string message);
    
private:
    void initialPosition();
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

#endif //LOWLEVELDRIVER_H_
