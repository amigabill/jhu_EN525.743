/***************************************************************
 * Name:      wxSmartHomeServerMain.h
 * Purpose:   Defines Application Frame
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner ()
 * License:
 **************************************************************/

#ifndef WXSMARTHOMESERVERMAIN_H
#define WXSMARTHOMESERVERMAIN_H

//(*Headers(wxSmartHomeServerFrame)
#include <wx/menu.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
//*)

class wxSmartHomeServerFrame: public wxFrame
{
    public:

        wxSmartHomeServerFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~wxSmartHomeServerFrame();

    private:

        //(*Handlers(wxSmartHomeServerFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        //*)

        //(*Identifiers(wxSmartHomeServerFrame)
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(wxSmartHomeServerFrame)
        wxStatusBar* StatusBar1;
        //*)

        DECLARE_EVENT_TABLE()
};

#endif // WXSMARTHOMESERVERMAIN_H
