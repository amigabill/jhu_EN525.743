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
//    wxString wxPortName = portName;

    sprintf(_shSerialPortName, "%s", portName);
    _shSerialPortFD = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    _FILEshSerialPortRX = fopen(portName, "r");
    _FILEshSerialPortTX = fopen(portName, "w");

//    g_shSerialPortZBwxFile.Open( wxPortName, wxFile::read_write);

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

//    _shSerialPortFD = open(_shSerialPortName, O_RDWR | O_NOCTTY | O_NDELAY);

    sprintf( sttyCmd, "stty -F %s %d cs8 cread clocal", _shSerialPortName, baud_rate );
    system(sttyCmd);
//    system("stty -F /dev/ttyUSB0 9600 cs8 cread clocal");

//    _shSerialPortZBwxFile.Open(SH_SERIAL_ZB_FILENAME, wxFile::read_write);

    uint8_t zbBufferTX[30] = {0x7E, 0x00, 0x1A, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x01, 0x01, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0xAE };
    write(_shSerialPortFD, (char *)zbBufferTX, 30);//write to port

//g_shSerialPortZBwxFile.Write(zbBufferTX, 30);

    return true;
}


// stop the serial port and close it out
void shSerialPort::stop(void)
{
    // close the serial port
    close(_shSerialPortFD);
    fclose(_FILEshSerialPortTX);
    fclose(_FILEshSerialPortRX);

//    _shSerialPortZBwxFile.Close();
}


// say if rxBuffer contains a new Zigbee frame/SmartHome message
// will be used? or use variable?
bool shSerialPort::rxAvailable(void)
{
    int bytes_avail = 0;
//    wxFileOffset bytes_avail = 0;

    ioctl(_shSerialPortFD, FIONREAD, &bytes_avail);

//    bytes_avail = _shSerialPortZBwxFile.Length();
    return( bytes_avail > 0 );
}


// receive a single byte form serial port
// serial port should already be open and configured as a file descriptor style access
// from one of /dev/tty* devices
uint8_t rxReceive(void)
{
    uint8_t charRX[1];

//    _shSerialPortZBwxFile.Read(charRX, (size_t)1);

    return( charRX[0] );
}


// transmit the content of the given message via serial port.
// Serial port should be already open and configured as a file descriptor style access
// to one of /dev/tty* devices
uint8_t shSerialPort::txSend(uint8_t charTX)
{
    //write(_shSerialPortFD, &charTX, 1);//write to port
    uint8_t zbBufferTX[30] = {0x7E, 0x00, 0x1A, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x01, 0x01, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0xAE };
    write(_shSerialPortFD, (char *)zbBufferTX, 30);//write to port
//    _shSerialPortZBwxFile.Write(zbBufferTX, 30);
}
