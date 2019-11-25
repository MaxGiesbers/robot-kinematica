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
  /**
   * @brief Construct a new Low Level Driver object
   * 
   * @param port 
   */
  LowLevelDriver(const std::string port);
  
  /**
   * @brief Destroy the Low Level Driver object
   * 
   */
  virtual ~LowLevelDriver();

  /**
   * @brief Writes a pwm command to the al5d_arm
   * 
   * @param message pwm command in al5d format
   */
  void writeMessage(const std::string& message);

private:
  /**
   * @brief for reading and writing to the al5d_arm
   * 
   */
  boost::asio::io_service io;

  /**
   * @brief serial port object for writing and reading to the al5d_arm
   * 
   */
  boost::asio::serial_port serial;

  /**
   * @brief Function for moving the al5d to its initial position (park)
   * 
   */
  void initialPosition();
};

#endif  // LOWLEVELDRIVER_H_
