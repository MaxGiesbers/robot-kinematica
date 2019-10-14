#include "al5d_controller/LowLevel/LowLevelDriver.h"
namespace Lowlevel {
LowLevelDriver::LowLevelDriver(const std::string port): serial(port)
{
    initialPosition();
}

LowLevelDriver::~LowLevelDriver()
{
    
}

void LowLevelDriver::goToPosition(std::vector<int>& positions, short time)
{
    std::stringstream ss;
    for(int i =0; i < positions.size(); ++i)
    {
        ss << "#" << i << "P" << positions[i];
    }

    ss << "T" << time << "\r";
    ROS_DEBUG_STREAM(ss.str());
    serial.writeString(ss.str());
}

void LowLevelDriver::initialPosition()
{
    ros::Duration(1).sleep();


    serial.writeString("#0 P1500 S500 #1 P1998 S500 #2 P1992 S500 #3 P1000 S500 #4 P1500 S500 #5 P1500 S500\r");
    ROS_INFO("STATE: INIT");
}

void LowLevelDriver::emergencyStop()
{
    serial.writeString("STOP 1\rSTOP 2\rSTOP 3\rSTOP 4\r");
    ROS_INFO("STATE: STOP");
    initialPosition();
}
}