#include "Al5dController/LowLevel/LowLevelDriver.h"

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

void LowLevelDriver::writeMessage(const std::string& message)
{
  ROS_DEBUG_STREAM(message);
  boost::asio::write(serial, boost::asio::buffer(message.c_str(), message.size()));
}

void LowLevelDriver::initialPosition()
{
  writeMessage("#0P1489#1P1798#2P1743#3P935#4P1500#5P1000T2000\r");
  ROS_INFO("STATE: INIT");
}