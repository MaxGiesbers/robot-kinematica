#include "al5d_controller/LowLevel/LowLevelDriver.h"

namespace
{
const int16_t BAUD_RATE = 9600;
}

LowLevelDriver::LowLevelDriver(const std::string port) : io(), serial(io, port)
{
  serial.set_option(boost::asio::serial_port_base::baud_rate(BAUD_RATE));
  serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));
  initialPosition();
}

LowLevelDriver::~LowLevelDriver()
{
}

void LowLevelDriver::goToPosition(std::vector<int>& positions, short time)
{
  std::stringstream ss;
  for (std::size_t i = 0; i < positions.size(); ++i)
  {
    ss << "#" << i << "P" << positions[i];
  }

  ss << "T" << time << "\r";
  ROS_DEBUG_STREAM(ss.str());
  writeString(ss.str());
}

void LowLevelDriver::initialPosition()
{
  ros::Duration(1).sleep();

  writeString("#0 P1500 S500 #1 P1998 S500 #2 P1992 S500 #3 P1000 S500 #4 P1500 S500 #5 P1500 S500\r");
  ROS_INFO("STATE: INIT");
}

void LowLevelDriver::writeString(std::string message)
{
  boost::asio::write(serial, boost::asio::buffer(message.c_str(), message.size()));
}

void LowLevelDriver::emergencyStop()
{
  writeString("STOP 1\rSTOP 2\rSTOP 3\rSTOP 4\r");
  ROS_INFO("STATE: STOP");
  initialPosition();
}
