#ifndef SH_LINUX_SERIAL_PORT
#define SH_LINUX_SERIAL_PORT

/***************************************************************
 * Name:      shLinuxSerialPort.h
 * Purpose:   Defines Application Frame
 *            This file provides a simpler serial port API for Linux, making use of Boost::asio calls
 *            This will encapsulate many details of configuring serial port for the SmartHome Xbee/Zigbee module
 *            running at 9600 8N1
 *            Use async_read into a single-byte buffer to avoid blocking on a serial read which may not happen for a long time.
 *
 *            Based on boost-asio examples at
 *            https://gist.github.com/yoggy/3323808
 *            http://www.webalice.it/fede.tft/serial_port/serial_port.html
 *            http://stackoverflow.com/questions/26267997/boost-asio-async-read-on-serial-port-completes-unexpectedly
 *
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-25
 * Copyright: Bill Toner (2015)
 * License:
 **************************************************************/

#include <stdint.h>

// Boost Asio library for communications things, such as serial port for Zigbee
#include <boost/asio.hpp>

#if 0
//#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#endif // 0

#include "SmartHome_Zigbee.h"

//#define SH_SERVER_SERIAL_PORT_NAME  (std::string)"/dev/ttyUSB0"
#define SH_SERVER_SERIAL_PORT_NAME  "/dev/ttyUSB0"
#define SH_SERVER_SERIAL_BAUD_RATE  9600
#define SH_SERVER_SERIAL_CHAR_SIZE  8
#define SH_SERVER_SERIAL_STOP_BITS  boost::asio::serial_port_base::stop_bits::one
#define SH_SERVER_SERIAL_PARITY     boost::asio::serial_port_base::parity::none
#define SH_SERVER_SERIAL_FLOW_CTRL  boost::asio::serial_port_base::flow_control::none

//typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;


class shSerialPort
{
    public:
//        uint8_t newTxReady = 0;
//        uint8_t newRxReady = 0;
        uint8_t ZBfrmReadyTX = 0; // a Zigbee frame is ready to transmit
        uint8_t ZBfrmReadyRX = 0; // a Zigbee frame has been received

        // constructor
//        shSerialPort(void);
//        shSerialPort(std::string port, unsigned int baud_rate) : io(), serial(io,port);
        shSerialPort(std::string port, unsigned int baud_rate);

        // destructor
        ~shSerialPort(void);

        // start the serial port, of the given port name and given baud rate
        bool start(std::string port, unsigned int baud_rate);

        // stop the serial port and close it out
        void stop(void);

//        // say if rxBuffer contains a new Zigbee frame/SmartHome message
//        uint8_t rxAvailable(void);

//        // read a byte
//        uint read(void);

        // transmit the content of
        uint8_t txSend(void);

    private:
//        boost::asio::io_service _ioService;
//        boost::asio::serial_port _shSerial(_ioService, SH_SERVER_SERIAL_PORT_NAME); // serial port class instance
////        char end_of_line_char_;
            boost::asio::io_service _ioService;
            boost::asio::serial_port _shSerial;

        uint8_t _txBuffer[ZB_TX_FRM_BYTES];  // buffer to hold Zigbee frame/SmartHome message to send
        uint8_t _rxBuffer[ZB_RX_FRM_BYTES];  // buffer to receive a Zigbee frame/Smarthome message into

        // callback function for boost library's async_read to call upon
        // we will do async_read with size of 1 byte, check that byte, and if a Zigbee delimiter char then read the
        // remainder of the Zigbee frame. If the first char is NOT a Zigbee delimiter, then ignore it.
        void _async_read_callback(const boost::system::error_code& error, std::size_t bytes_transferred);

};

#endif // SH_LINUX_SERIAL_PORT
