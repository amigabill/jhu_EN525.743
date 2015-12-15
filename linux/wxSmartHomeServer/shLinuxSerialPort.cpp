/***************************************************************
 * Name:      shLinuxSerialPort.cpp
 * Purpose:   Defines Application Frame
 *            This file provides a simpler serial port API for Linux, making use of Boost::asio calls
 *            This will encapsulate many details of configuring serial port for the SmartHome Xbee/Zigbee module
 *            running at 9600 8N1
 *            Use async_read into a single-byte buffer to avoid blocking on a serial read which may not happen for a long time.
 *
 *            non-blocking tty serial read example at http://www.cmrr.umn.edu/~strupp/serial.html
 *
 *
 *            Inspired by references and examples at
 *            http://linux.about.com/od/commands/l/blcmdl1_stty.htm
 *            http://www.cmrr.umn.edu/~strupp/serial.html
 *
 *         // did not end up using Boost-Asio, but keep examples here for future reference
 *         // ignore boost comments for now
 *         // This file provides a simpler serial port API for Linux, making
 *         // of Boost::asio calls.
 *         // This will encapsulate many details of configuring serial port
 *         // for the SmartHome Xbee/Zigbee module running at 9600 8N1
 *         // Use async_read into a single-byte buffer to avoid blocking on
 *         // a serial read which may not happen for a long time.
 *         //
 *         // Based on examples at
 *         // http://www.cmrr.umn.edu/~strupp/serial.html#5_1
 *
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner (2015)
 * License:   This library is free software; you can redistribute it and/or
 *            modify it under the terms of the GNU Lesser General Public
 *            License as published by the Free Software Foundation; either
 *            version 2.1 of the License, or (at your option) any later version.
 *            http://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
 * **************************************************************/


#include <wx/log.h>

#include "SmartHomeServerAppDetails.h"

#include "shLinuxSerialPort.h"

#include <iostream>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

// constructor
shSerialPort::shSerialPort(void)
{
}


// destructor
shSerialPort::~shSerialPort()
{
    stop();
}


// start the serial port, of the given port name and given baud rate
// TODO - cleanup
bool shSerialPort::start(const char* portName, unsigned int baud_rate)
{
    char sttyCmd[100]; // temp string to hold system command to set serial port for use
    //wxString sttyCmd; // temp string to hold system command to set serial port for use

    sprintf(_shSerialPortName, "%s", portName);

    shSerialPortFD = open(portName, O_RDWR | O_NOCTTY | O_NDELAY); // | O_NONBLOCK);

    if(shSerialPortFD == -1) // if open is unsucessful
	{
        wxLogMessage("open_port: Unable to open %s. \n", portName) ;
//        wxLogMessage("open_port: Unable to open %s. \n", "/dev/ttyUSB0") ;
	}
	else
	{
		fcntl(shSerialPortFD, F_SETFL, FNDELAY);  // FNDELAY means non-blocking, return 0 bytes received if buffer empty
//		fcntl(shSerialPortFD, F_SETFL, 0); // 0 here leaves it in a blocking state, which is not desired in this App

//        wxLogMessage("port %s is open.\n", portName) ;
	}

    // prepare configuration settings for the serial port
    sprintf( sttyCmd, "stty -F %s %d raw cs8 cread clocal time 1", _shSerialPortName, baud_rate );
//    sttyCmd = wxT("stty -F ") + _shSerialPortName + wxT(" ") + wxString::Format(wxT("%d"), baud_rate) + wxT(" raw cs8 cread clocal time 1");

//    wxLogMessage( "sttyCmd = %s", sttyCmd ) ;

    // do a "system" call to set the serial port configuration settings
//    system( sttyCmd.ToAscii() ); //wxString
    system( sttyCmd );

//    _shSerialPortZBwxFile.Open(SH_SERIAL_ZB_FILENAME, wxFile::read_write);

    return true;
}


// stop the serial port and close it out
void shSerialPort::stop(void)
{
    // close the serial port
    close(shSerialPortFD);
//    _shSerialPortZBwxFile.Close();
}


// say if rxBuffer contains a new Zigbee frame/SmartHome message
// will be used? or use variable?
bool shSerialPort::rxAvailable(void)
{
    // iniital test code implemented, not yet actually used

    int bytes_avail = 0;
//    wxFileOffset bytes_avail = 0;

//    ioctl(_shSerialPortFD, FIONREAD, &bytes_avail);
    ioctl(shSerialPortFD, FIONREAD, &bytes_avail);

//    bytes_avail = _shSerialPortZBwxFile.Length();
    return( bytes_avail > 0 );
}


// receive a single byte form serial port
// serial port should already be open and configured as a file descriptor style access
// from one of /dev/tty* devices
// this should be NON-blocking in case there are no Zigbee messages coming in for a long time.
uint16_t shSerialPort::rxReceive(void)
{
    // iniital test code implemented, not yet actually used

    uint16_t  numBytes_dataByte = 0; // 16bit thing to hold the number of bytes received (0 or 1) and data byte to send back
    uint16_t  *ptr_numBytes_dataByte = &numBytes_dataByte;
    int       bytes_read = 0;
    uint8_t   charRX[1];

//    _shSerialPortZBwxFile.Read(charRX, (size_t)1); // if tty is wxFile
    bytes_read= read(shSerialPortFD, charRX, (size_t)1);

    ptr_numBytes_dataByte[1] = bytes_read;
    ptr_numBytes_dataByte[0] = charRX[0];

//    return( charRX[0] );
    return( numBytes_dataByte );
}


// transmit the content of the given message via serial port.
// Serial port should be already open and configured as a file descriptor style access
// to one of /dev/tty* devices
uint8_t shSerialPort::txSend(uint8_t charTX)
{
    //write(_shSerialPortFD, &charTX, 1);//write to port

    // iniital test code implemented, not yet actually used

    uint8_t zbBufferTX[30] = {0x7E, 0x00, 0x1A, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x01, 0x01, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0xAE };
//    write(_shSerialPortFD, (char *)zbBufferTX, 30);//write to port
    write(shSerialPortFD, (char *)zbBufferTX, 30);//write to port
//    _shSerialPortZBwxFile.Write(zbBufferTX, 30);

    return(true);
}

