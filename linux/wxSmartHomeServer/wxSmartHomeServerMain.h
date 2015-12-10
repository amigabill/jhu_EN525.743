/***************************************************************
 * Name:      wxSmartHomeServerMain.h
 * Purpose:   Defines Application Frame
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner (2015)
 * License:
 * Initially generated by wxSmith GUI builder tool
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
#include <wx/file.h>
//*)


#include <stdint.h>

#include "SmartHomeServerAppDetails.h"

#include "SmartHome_Zigbee_Linux.h"
#include "SmartHome_NodeInfo.h"

#include "shLinuxSerialPort.h"


class wxSmartHomeServerFrame: public wxFrame
{
    public:
        uint16_t    shThisNodeID = 0;

        wxTextCtrl* shTextCtrlEvtlog;
        wxTextCtrl* shTextCtrlCurrentDate;
        wxTextCtrl* shTextCtrlCurrentTime;
        wxGauge*    shIntensityGauge;

//        SHzigbee mySHzigbee = SHzigbee();

        //#define SH_DEBUG_LOG_FILENAME  "/home/billt/shDebugLog.log"
        const wxString SH_DEBUG_LOG_FILENAME  = "/home/billt/shDebugLog.log";
//        const wxString *PTR_SH_DEBUG_LOG_FILENAME = &SH_DEBUG_LOG_FILENAME;
        wxFile shDebugLogFile;

//        const char* SH_SERIAL_ZB_FILENAME  = "/dev/ttyUSB0";
//        shSerialPort  shServerSerialPort = shSerialPort( SH_SERIAL_ZB_FILENAME );
//        shSerialPort  shServerSerialPort = shSerialPort();

        // nodeInfo struct of the currently selected target load to be controlled
        SHnodeInfo  shCurrentLoadNodeInfo;

        wxSmartHomeServerFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~wxSmartHomeServerFrame();

        // Additional SmartHome functions
        void    SHguageDrawCurrentIntensity(void);

    private:

//        const char* _SH_SERVER_SERIAL_PORT_NAME = "/dev/ttyUSB0";
//        const int   _SH_SERVER_SERIAL_BAUD_RATE = 9600;
//        const int   _SH_SERVER_SERIAL_CHAR_SIZE =  8;


        uint8_t     shCurrentIntensity;
//        uint16_t    shThisNodeID = 0;
        uint8_t    *ptrThisNodeID = (uint8_t *)&shThisNodeID;
        uint8_t     shThisNodeType;         // 0=ctrl, 1=light, 2=fan
        FILE       *FILEshEventsLog;


        //(*Handlers(wxSmartHomeServerFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
//        void OnIdle(wxIdleEvent &event);
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
        void      SHinitServerNodeInfo(void);
        void      SHinitLoadNodeInfo(void);
        uint16_t  SHgetLoadNodeIDfromStorage(uint16_t roomNum, uint8_t loadNum);
        uint8_t   SHgetLoadNodeTypefromStorage(uint16_t roomNum, uint8_t loadNum);


        //(*Identifiers(wxSmartHomeServerFrame)
        static const long ID_TEXTCTRL_SH_EVT_LOG;
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
        static const long ID_TEXTCTRL_SH_CURTIME; //ID_TEXTCTRL2;
        static const long ID_TEXTCTRL_SH_CURDATE;
        static const long ID_TEXTCTRL_SH_ROOMNAME;
        static const long ID_TEXTCTRL_SH_LOADNAME;
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
//        wxStaticText* shStaticTextRoomName;
        wxTextCtrl* shTextCtrlRoomName;
        wxStaticText* StaticText1;
        wxBitmapButton* shBMPbtnRoomLeft;
        wxBitmapButton* shBMPbuttonFAV;
        wxBitmapButton* shBMPbtnRoomRight;
        wxBitmapButton* shBMPbtnLoadLeft;
        wxStaticLine* StaticLine1;
        wxPanel* shPanel1;
        wxBitmapButton* shBMPbuttonOFF;
        wxBitmapButton* shBMPbuttonDWN;
        wxStaticBitmap* StaticBitmap3;
        wxStaticBitmap* StaticBitmap1;
//        wxStaticText* shStaticTextLoadName;
        wxTextCtrl* shTextCtrlLoadName;
        wxStaticBitmap* StaticBitmap4;
        wxBitmapButton* shBMPbuttonUP;
        wxStaticBitmap* StaticBitmap2;
        //*)



        DECLARE_EVENT_TABLE()
};

#endif // WXSMARTHOMESERVERMAIN_H