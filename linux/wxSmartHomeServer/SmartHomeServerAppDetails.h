#ifndef SMARTHOMESERVERAPPDETAILS_H_INCLUDED
#define SMARTHOMESERVERAPPDETAILS_H_INCLUDED

// This file does some #defines or constant definitions for
// inclusion throughout this project's source files.

#include <stdint.h>

#include <wx/string.h>


#define LINUX

//#define SH_SERV_APP_NAME "wxSmartHomeServer"
const wxString SH_SERV_APP_NAME      = "wxSmartHomeServer";
const wxString SH_SERV_APP_PATH_ROOT = "/home/smarthome/.wxSmartHome";

const wxString SH_CTRL_EVENT_LOG_FILENAME = "/home/smarthome/.wxSmartHome/shEvents.log";

//const char* SH_SERVER_SERIAL_PORT_NAME = "/dev/ttyUSB0";
//const int   SH_SERVER_SERIAL_BAUD_RATE = 9600;



#endif // SMARTHOMESERVERAPPDETAILS_H_INCLUDED
