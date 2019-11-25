#ifndef LOWLEVELDRIVER_H
#define LOWLEVELDRIVER_H

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

private:
  boost::asio::io_service io;
  boost::asio::serial_port serial;
  void initialPosition();
};

#endif  // LOWLEVELDRIVER_H_
