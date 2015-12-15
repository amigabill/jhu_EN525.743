#ifndef SMARTHOMESERVERAPPDETAILS_H_INCLUDED
#define SMARTHOMESERVERAPPDETAILS_H_INCLUDED

/***************************************************************
 * Name:      SmartHomeServerAppDetails.h
 * Purpose:   Defines constant values for project Linux "gateway" node
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner (2015)
 * License:   This library is free software; you can redistribute it and/or
 *            modify it under the terms of the GNU Lesser General Public
 *            License as published by the Free Software Foundation; either
 *            version 2.1 of the License, or (at your option) any later version.
 *            http://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
 * **************************************************************/


// This file does some #defines or constant definitions for
// inclusion throughout this project's source files.

#include <stdint.h>

#include <wx/string.h>


#define LINUX

const wxString SH_SERV_APP_NAME            = "wxSmartHomeServer";
const wxString SH_SERV_APP_PATH_ROOT       = "/home/smarthome/.wxSmartHome/";
//const wxString defaultSHpathConfig         = "/home/smarthome/.wxSmartHome/CONFIG/";
const wxString SH_SERV_PATH_CFG_DEFAULT    = SH_SERV_APP_PATH_ROOT+ "CONFIG/";
const wxString SH_SERV_PATH_ROOMS_DEFAULT  = SH_SERV_PATH_CFG_DEFAULT + "ROOMS/";

const wxString SH_CTRL_EVENT_LOG_FILENAME = "/home/smarthome/.wxSmartHome/shEvents.log";

//const char* SH_SERVER_SERIAL_PORT_NAME = "/dev/ttyUSB0";
//const int   SH_SERVER_SERIAL_BAUD_RATE = 9600;

#define NUM_ROOMS_MAX  65535
#define NUM_LOADS_MAX  65535



#endif // SMARTHOMESERVERAPPDETAILS_H_INCLUDED
