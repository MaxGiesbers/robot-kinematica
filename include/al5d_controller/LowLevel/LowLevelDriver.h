#ifndef LOWLEVELDRIVER_H_
#define LOWLEVELDRIVER_H_

#include <ros/ros.h>
#include "Servo.h"
#include <string>
#include <vector>
#include <boost/asio.hpp>

class LowLevelDriver
{
public:
  LowLevelDriver(const std::string port);
  virtual ~LowLevelDriver();

  void writeMessage(const std::string& message);
  void emergencyStop();

private:
  boost::asio::io_service io;
  boost::asio::serial_port serial;
};

#endif  // LOWLEVELDRIVER_H_
