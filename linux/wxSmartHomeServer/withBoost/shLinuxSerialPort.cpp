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

#include "shLinuxSerialPort.h"

 #include <iostream>

// constructor
//shSerialPort::shSerialPort(std::string portName): _ioService()
//shSerialPort::shSerialPort(void): _ioService()
//shSerialPort::shSerialPort(void)
//shSerialPort::shSerialPort(void): end_of_line_char_('\n')
//: io(), serial(io,port)
shSerialPort::shSerialPort(std::string portName, unsigned int baud_rate) : _ioService(), _shSerial(_ioService,portName)
{
//	_shSerial = serial_port_ptr(new boost::asio::serial_port(_ioService));
//	_shSerial.open(portName, ec);
         _shSerial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}


// destructor
shSerialPort::~shSerialPort(void)
{
    stop();
}


// start the serial port, of the given port name and given baud rate
bool shSerialPort::start(std::string portName, unsigned int baud_rate)
{
    boost::system::error_code ec;

    // open the serial port and set baud rate

//    if ( shSerialPort._shSerial )
    if ( _shSerial.is_open() )
    {
		std::cout << "error : port is already opened..." << std::endl;
		return false;
	}

//	_shSerial = serial_port_ptr(new boost::asio::serial_port(_ioService));
	_shSerial.open(portName, ec);
	if (ec)
    {
		std::cout << "error : port_->open() failed...portName="
			<< portName << ", e=" << ec.message().c_str() << std::endl;
		return false;
	}

	// option settings...
	_shSerial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	_shSerial.set_option(boost::asio::serial_port_base::character_size(SH_SERVER_SERIAL_CHAR_SIZE));
	_shSerial.set_option(boost::asio::serial_port_base::stop_bits(SH_SERVER_SERIAL_STOP_BITS));
	_shSerial.set_option(boost::asio::serial_port_base::parity(SH_SERVER_SERIAL_PARITY));
	_shSerial.set_option(boost::asio::serial_port_base::flow_control(SH_SERVER_SERIAL_FLOW_CTRL));

//    boost::thread t(boost::bind(&boost::asio::io_service::run, &_ioService));

    return true;
}


// stop the serial port and close it out
void shSerialPort::stop(void)
{
    // close the serial port
//	boost::mutex::scoped_lock look(mutex_);

	if ( _shSerial.is_open() ) {
		_shSerial.cancel();
		_shSerial.close();
//		_shSerial.reset();
	}

	_ioService.stop();
	_ioService.reset();
}


#if 0
// say if rxBuffer contains a new Zigbee frame/SmartHome message
// will be used? or use variable?
uint8_t shSerialPort::rxAvailable(void)
{

}
#endif


// transmit the content of
uint8_t shSerialPort::txSend(void)
{

}
