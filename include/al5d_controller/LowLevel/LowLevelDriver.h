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
   * @brief 
   * 
   * @param message 
   */
  void writeMessage(const std::string& message);

private:
  /**
   * @brief 
   * 
   */
  boost::asio::io_service io;

  /**
   * @brief 
   * 
   */
  boost::asio::serial_port serial;

  /**
   * @brief Function for moving the al5d to its initial position
   * 
   */
  void initialPosition();
};

#endif  // LOWLEVELDRIVER_H_
