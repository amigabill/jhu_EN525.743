/***************************************************************
 * Name:      wxSmartHomeServerApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner ()
 * License:
 **************************************************************/

#include "wxSmartHomeServerApp.h"

//(*AppHeaders
#include "wxSmartHomeServerMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(wxSmartHomeServerApp);

bool wxSmartHomeServerApp::OnInit()
{
    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	wxSmartHomeServerFrame* Frame = new wxSmartHomeServerFrame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}
