/***************************************************************
 * Name:      wxSmartHomeServerApp.h
 * Purpose:   Defines wxWidgets Application Class for project Linux "gateway" node
 * Purpose:   Code for Application Class for project Linux "gateway"
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner (2015)
 * License:   This library is free software; you can redistribute it and/or
 *            modify it under the terms of the GNU Lesser General Public
 *            License as published by the Free Software Foundation; either
 *            version 2.1 of the License, or (at your option) any later version.
 *            http://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
 *
 * Initially generated by wxSmith GUI builder tool
 **************************************************************/


#ifndef WXSMARTHOMESERVERAPP_H
#define WXSMARTHOMESERVERAPP_H

#include <wx/app.h>
#include <wx/file.h>
#include <wx/log.h>

#include <stdint.h>

#include "SmartHomeServerAppDetails.h"

#include "wxSmartHomeServerMain.h"

#include "SmartHome_Zigbee_Linux.h"

const char* SH_SERVER_SERIAL_PORT_NAME = "/dev/ttyUSB0";
const int   SH_SERVER_SERIAL_BAUD_RATE = 9600;


class wxSmartHomeServerApp : public wxApp
{
    public:
        virtual bool OnInit();
        void OnIdle(wxIdleEvent &event);

    private:
        uint8_t  _appIsReady = NO;
        wxSmartHomeServerFrame* Frame;
        size_t   _shLogFileNumLinesPrev = 0; // number of lines in the SmartHome Control Event log file from the last time we checked
        size_t   _shLogFileLengthPrev = 0;   // number of bytes in the SmartHome Control Event log file from the last time we checked
        wxString _shCurrentDateTime;
        wxString _shCurrentDay;
        wxString _shCurrentMonth;
        wxString _shCurrentDate;
        wxString _shCurrentYear;
        wxString _shCurrentFullDate;
        wxString _shCurrentDayFullDate;
        wxString _shCurrentTime;

//        SHzigbee mySHzigbee = SHzigbee();
        SHzigbee _mySHzigbee;


        bool _SHupdateGUIlogText(const wxString& shLogFileName);
        uint8_t _SHupdateGUItimeText(void);

        uint8_t _SHreceiveZBserialData(void);
        uint8_t _SHsendZBserialData(void);

        void _doServerNodeIDmsgSM(void);
        bool _logSHcmdEventIfCompleted(void);

        void _shGUIupdateLoadIntensityLevel(void);
        void _SHnewMsgUpdateLoadNodeInfo(void);


        DECLARE_EVENT_TABLE()

};

#endif // WXSMARTHOMESERVERAPP_H
