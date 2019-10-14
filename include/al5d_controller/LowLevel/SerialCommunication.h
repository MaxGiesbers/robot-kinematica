#ifndef SERIALCOMMUNICATION_H_
#define SERIALCOMMUNICATION_H_

/**
 * @brief The Serial communication that finds place between the low level interface and the AL5D robot.
 * 
 * @file SerialCommunication.h
 * @author Joris van Zeeland, Max Giesbers
 * @date 2018-09-25
 */

#include <boost/asio.hpp>
#include <string>

namespace Lowlevel {
class SerialCommunication
{
public:
    /**
     * @brief Construct a new Serial Communication object
     * 
     * @param port contains a string with the port name of the usb port.
     * @param baud_rate contains unsigned int with the baud rate of the serial communication.
     */
    SerialCommunication(std::string port);
    /**
     * @brief Destroy the Serial Communication object
     * 
     */
    virtual ~SerialCommunication();
    /**
     * @brief Sends strings with boost asio write.
     * 
     * @param message that contains a string that will be send with boost asio.
     */
    void writeString(std::string message);

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    const short baudRate = 9600;
};
}

#endif // SERIALCOMMUNICATION_H_