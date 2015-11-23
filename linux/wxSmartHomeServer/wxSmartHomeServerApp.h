/***************************************************************
 * Name:      wxSmartHomeServerApp.h
 * Purpose:   Defines Application Class
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner ()
 * License:
 **************************************************************/

#ifndef WXSMARTHOMESERVERAPP_H
#define WXSMARTHOMESERVERAPP_H

#include <wx/app.h>


#define SH_SERV_APP_NAME "wxSmartHomeServer"

class wxSmartHomeServerApp : public wxApp
{
    public:
        virtual bool OnInit();
};

#endif // WXSMARTHOMESERVERAPP_H
