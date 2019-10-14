#include "al5d_controller/LowLevel/SerialCommunication.h"
#include <iostream>
namespace Lowlevel {
SerialCommunication::SerialCommunication(std::string port)
: io(), serial(io,port)
{
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));
}

void SerialCommunication::writeString(std::string message)
{
    std::cout << message << std::endl;
    boost::asio::write(serial,boost::asio::buffer(message.c_str(),message.size()));
}

SerialCommunication::~SerialCommunication()
{
}
}