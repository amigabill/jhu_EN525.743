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
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

// constructor
//shSerialPort::shSerialPort(std::string portName, unsigned int baud_rate)
shSerialPort::shSerialPort(const char* portName)
{
    sprintf(_shSerialPortName, "%s", portName);
//    shSerialPortFD = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    _FILEshSerialPortRX = fopen(portName, "r");
    _FILEshSerialPortTX = fopen(portName, "w");

//	FILE *input;
//	FILE *output;
//	input  = fopen(portName, "r");	  //open the terminal keyboard
//  output = fopen(portName, "w");

    // set given serial port to baud rate
//    sprintf( sttyCmd, "stty -F %s %s cs8 cread clocal", portName, baud_rate );
//    system(sttyCmd);

#if 0
if(shSerialPortFD == -1) // if open is unsucessful
	{
		printf("open_port: Unable to open %s. \n", portName);
	}
	else
	{
		fcntl(shSerialPortFD, F_SETFL, 0);
		printf("port %s is open.\n" portName);
	}
#endif
}


// destructor
shSerialPort::~shSerialPort(void)
{
    stop();
}


// start the serial port, of the given port name and given baud rate
//bool shSerialPort::start(std::string portName, unsigned int baud_rate)
//bool shSerialPort::start(char* portName, unsigned int baud_rate)
bool shSerialPort::start(unsigned int baud_rate)
{
    char sttyCmd[100]; // temp string to hold system command to set serial port for use

    //shSerialPortName
//    sprintf( _shSerialPortName, "%s", portName);

    _shSerialPortFD = open(_shSerialPortName, O_RDWR | O_NOCTTY | O_NDELAY);

    sprintf( sttyCmd, "stty -F %s %d cs8 cread clocal", _shSerialPortName, baud_rate );
    system(sttyCmd);
//    system("stty -F /dev/ttyUSB0 9600 cs8 cread clocal");

    return true;
}


// stop the serial port and close it out
void shSerialPort::stop(void)
{
    // close the serial port
    close(_shSerialPortFD);
    fclose(_FILEshSerialPortTX);
    fclose(_FILEshSerialPortRX);
}


#if 1
// say if rxBuffer contains a new Zigbee frame/SmartHome message
// will be used? or use variable?
bool shSerialPort::rxAvailable(void)
{
    int bytes_avail = 0;

    ioctl(_shSerialPortFD, FIONREAD, &bytes_avail);

    return( bytes_avail > 0 );
}
#endif


uint rxReceive(void)
{
    char charRX;

    return();
}

// transmit the content of
uint8_t shSerialPort::txSend(char charTX)
{
    write(_shSerialPortFD, &charTX, 1);//write to port
}
