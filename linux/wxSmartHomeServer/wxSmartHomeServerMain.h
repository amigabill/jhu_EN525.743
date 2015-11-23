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
#include <wx/gauge.h>
#include <wx/bmpbuttn.h>
#include <wx/checkbox.h>
#include <wx/sizer.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/statusbr.h>
#include <wx/statline.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/statbmp.h>
#include <wx/log.h>
//*)


#include <stdint.h>

#include "SmartHome_Zigbee.h"
#include "SmartHome_NodeInfo.h"


class wxSmartHomeServerFrame: public wxFrame
{
    public:

        wxSmartHomeServerFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~wxSmartHomeServerFrame();

    private:

        uint8_t     shCurrentIntensity;
        uint16_t    shThisNodeID = 0;
        uint8_t    *ptrThisNodeID = (uint8_t *)&shThisNodeID;
        uint8_t     shThisNodeType;         // 0=ctrl, 1=light, 2=fan

        // nodeInfo struct of the currently selected target load to be controlled
        SHnodeInfo  shCurrentLoadNodeInfo;


        //(*Handlers(wxSmartHomeServerFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnshBMPbtnRoomRightClick(wxCommandEvent& event);
        void OnshBMPbtnRoomLeftClick(wxCommandEvent& event);
        void OnshBMPbtnLoadRightClick(wxCommandEvent& event);
        void OnshBMPbtnLoadLeftClick(wxCommandEvent& event);
        void OnshBMPbtnIncIntClick(wxCommandEvent& event);
        void OnshBMPbtnFavIntClick(wxCommandEvent& event);
        void OnshBMPbtnDecIntClick(wxCommandEvent& event);
        void OnshBMPbtnOnClick(wxCommandEvent& event);
        void OnshBMPbtnOffClick(wxCommandEvent& event);
        //*)

        // Additional SmartHome functions
        void      SHguageDrawCurrentIntensity(void);
        void      SHinitServerNodeInfo(void);
        void      SHinitLoadNodeInfo(void);
        uint16_t  SHgetLoadNodeIDfromStorage(uint16_t roomNum, uint8_t loadNum);
        uint8_t   SHgetLoadNodeTypefromStorage(uint16_t roomNum, uint8_t loadNum);


        //(*Identifiers(wxSmartHomeServerFrame)
        static const long ID_TEXTCTRL1;
        static const long ID_GAUGE_SH_LD;
        static const long ID_BITMAPBUTTON_SH_UP;
        static const long ID_BITMAPBUTTON_SH_FAV;
        static const long ID_BITMAPBUTTON_SH_DWN;
        static const long ID_STATICTEXT1;
        static const long ID_STATICLINE1;
        static const long ID_BITMAPBUTTON_SH_ON;
        static const long ID_BITMAPBUTTON_SH_OFF;
        static const long ID_BITMAPBUTTON_SH_RM_LFT;
        static const long ID_BITMAPBUTTON_SH_LD_LFT;
        static const long ID_BITMAPBUTTON_SH_RM_RT;
        static const long ID_BITMAPBUTTON_SH_LD_RT;
        static const long ID_CHECKBOX_SH_HM1;
        static const long ID_CHECKBOX_SH_HM2;
        static const long ID_STATICTEXT_SH_RM_NAME;
        static const long ID_STATICTEXT_SH_LD_NAME;
        static const long ID_STATICBITMAP1;
        static const long ID_STATICBITMAP2;
        static const long ID_STATICBITMAP3;
        static const long ID_STATICBITMAP4;
        static const long ID_PANEL1;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(wxSmartHomeServerFrame)
        wxBitmapButton* shBMPbtnLoadRight;
        wxCheckBox* shCheckBoxIsHome2;
        wxStatusBar* StatusBar1;
        wxBitmapButton* shBMPbuttonON;
        wxCheckBox* shCheckBoxIsHome1;
        wxStaticText* shStaticTextRoomName;
        wxStaticText* StaticText1;
        wxBitmapButton* shBMPbtnRoomLeft;
        wxBitmapButton* shBMPbuttonFAV;
        wxBitmapButton* shBMPbtnRoomRight;
        wxBitmapButton* shBMPbtnLoadLeft;
        wxStaticLine* StaticLine1;
        wxPanel* shPanel1;
        wxBitmapButton* shBMPbuttonOFF;
        wxBitmapButton* shBMPbuttonDWN;
        wxTextCtrl* TextCtrl1;
        wxStaticBitmap* StaticBitmap3;
        wxStaticBitmap* StaticBitmap1;
        wxStaticText* shStaticTextLoadName;
        wxStaticBitmap* StaticBitmap4;
        wxGauge* shIntensityGauge;
        wxBitmapButton* shBMPbuttonUP;
        wxStaticBitmap* StaticBitmap2;
        //*)



        DECLARE_EVENT_TABLE()
};

#endif // WXSMARTHOMESERVERMAIN_H
