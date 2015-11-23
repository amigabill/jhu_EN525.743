/***************************************************************
 * Name:      wxSmartHomeServerMain.cpp
 * Purpose:   Code for Application Frame
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner ()
 * License:
 **************************************************************/

#include "wxSmartHomeServerMain.h"

//(*InternalHeaders(wxSmartHomeServerFrame)
#include "wx/defs.h"
#include "wx/wxprec.h"
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/msgdlg.h>
//*)


// basic file operations
#include <iostream>
#include <fstream>
using namespace std;

#include <stdio.h>
#define FILE_READ "r"


//helper functions
enum wxbuildinfoformat {
    short_f, long_f };


//#define __UNIX__
//#define wxUSE_UNICODE

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(wxSmartHomeServerFrame)
const long wxSmartHomeServerFrame::ID_TEXTCTRL1 = wxNewId();
const long wxSmartHomeServerFrame::ID_GAUGE_SH_LD = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_UP = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_FAV = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_DWN = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICTEXT1 = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICLINE1 = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_ON = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_OFF = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_RM_LFT = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_LD_LFT = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_RM_RT = wxNewId();
const long wxSmartHomeServerFrame::ID_BITMAPBUTTON_SH_LD_RT = wxNewId();
const long wxSmartHomeServerFrame::ID_CHECKBOX_SH_HM1 = wxNewId();
const long wxSmartHomeServerFrame::ID_CHECKBOX_SH_HM2 = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICTEXT_SH_RM_NAME = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICTEXT_SH_LD_NAME = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICBITMAP1 = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICBITMAP2 = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICBITMAP3 = wxNewId();
const long wxSmartHomeServerFrame::ID_STATICBITMAP4 = wxNewId();
const long wxSmartHomeServerFrame::ID_PANEL1 = wxNewId();
const long wxSmartHomeServerFrame::idMenuQuit = wxNewId();
const long wxSmartHomeServerFrame::idMenuAbout = wxNewId();
const long wxSmartHomeServerFrame::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(wxSmartHomeServerFrame,wxFrame)
    //(*EventTable(wxSmartHomeServerFrame)
    //*)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_RM_RT, wxSmartHomeServerFrame::OnshBMPbtnRoomRightClick)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_RM_LFT, wxSmartHomeServerFrame::OnshBMPbtnRoomLeftClick)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_LD_RT, wxSmartHomeServerFrame::OnshBMPbtnLoadRightClick)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_LD_LFT, wxSmartHomeServerFrame::OnshBMPbtnLoadLeftClick)

    EVT_BUTTON(ID_BITMAPBUTTON_SH_UP, wxSmartHomeServerFrame::OnshBMPbtnIncIntClick)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_FAV, wxSmartHomeServerFrame::OnshBMPbtnFavIntClick)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_DWN, wxSmartHomeServerFrame::OnshBMPbtnDecIntClick)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_ON, wxSmartHomeServerFrame::OnshBMPbtnOnClick)
    EVT_BUTTON(ID_BITMAPBUTTON_SH_OFF, wxSmartHomeServerFrame::OnshBMPbtnOffClick)

END_EVENT_TABLE()

wxSmartHomeServerFrame::wxSmartHomeServerFrame(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(wxSmartHomeServerFrame)
    wxMenuItem* MenuItem2;
    wxMenuItem* MenuItem1;
    wxMenu* Menu1;
    wxBoxSizer* BoxSizer1;
    wxMenuBar* MenuBar1;
    wxMenu* Menu2;

    Create(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    shPanel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxSize(840,480), wxNO_BORDER|wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    shPanel1->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_INFOBK));
    TextCtrl1 = new wxTextCtrl(shPanel1, ID_TEXTCTRL1, _("Text"), wxPoint(272,64), wxSize(560,368), wxTE_AUTO_SCROLL|wxTE_MULTILINE|wxTE_READONLY|wxTE_LEFT, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    TextCtrl1->SetMaxSize(wxSize(-1,-1));
    TextCtrl1->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_HIGHLIGHTTEXT));
    TextCtrl1->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_3DDKSHADOW));
    shIntensityGauge = new wxGauge(shPanel1, ID_GAUGE_SH_LD, 100, wxPoint(8,24), wxSize(28,172), wxGA_VERTICAL, wxDefaultValidator, _T("ID_GAUGE_SH_LD"));
    shIntensityGauge->SetValue(30);
    shBMPbuttonUP = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_UP, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/UP.BMP"))), wxPoint(64,16), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_UP"));
    shBMPbuttonFAV = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_FAV, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/FAV.BMP"))), wxPoint(48,80), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_FAV"));
    shBMPbuttonDWN = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_DWN, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/DOWN.BMP"))), wxPoint(64,152), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_DWN"));
    StaticText1 = new wxStaticText(shPanel1, ID_STATICTEXT1, _("  SmartHome Event Log  "), wxPoint(280,16), wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    StaticText1->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_HIGHLIGHTTEXT));
    StaticLine1 = new wxStaticLine(shPanel1, ID_STATICLINE1, wxPoint(272,40), wxSize(560,16), wxLI_HORIZONTAL, _T("ID_STATICLINE1"));
    StaticLine1->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_ACTIVECAPTION));
    shBMPbuttonON = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_ON, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/ON.bmp"))), wxPoint(152,16), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_ON"));
    shBMPbuttonOFF = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_OFF, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/OFF.BMP"))), wxPoint(152,112), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTONSH_OFF"));
    shBMPbtnRoomLeft = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_RM_LFT, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/RLBUTLFT.BMP"))), wxPoint(16,282), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_RM_LFT"));
    shBMPbtnLoadLeft = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_LD_LFT, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/RLBUTLFT.BMP"))), wxPoint(16,386), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_LD_LFT"));
    shBMPbtnRoomRight = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_RM_RT, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/RLBUTRIT.BMP"))), wxPoint(191,282), wxDefaultSize, wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_RM_RT"));
    shBMPbtnLoadRight = new wxBitmapButton(shPanel1, ID_BITMAPBUTTON_SH_LD_RT, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/RLBUTRIT.BMP"))), wxPoint(191,386), wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER, wxDefaultValidator, _T("ID_BITMAPBUTTON_SH_LD_RT"));
    shCheckBoxIsHome1 = new wxCheckBox(shPanel1, ID_CHECKBOX_SH_HM1, _("Home 1"), wxPoint(296,448), wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX_SH_HM1"));
    shCheckBoxIsHome1->SetValue(false);
    shCheckBoxIsHome2 = new wxCheckBox(shPanel1, ID_CHECKBOX_SH_HM2, _("Home 2"), wxPoint(392,448), wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX_SH_HM2"));
    shCheckBoxIsHome2->SetValue(false);
    shStaticTextRoomName = new wxStaticText(shPanel1, ID_STATICTEXT_SH_RM_NAME, _("Bedroom 1"), wxPoint(80,304), wxSize(120,20), wxALIGN_CENTRE|wxNO_BORDER, _T("ID_STATICTEXT_SH_RM_NAME"));
    shStaticTextRoomName->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_HIGHLIGHTTEXT));
    shStaticTextLoadName = new wxStaticText(shPanel1, ID_STATICTEXT_SH_LD_NAME, _("Fan 1"), wxPoint(80,410), wxSize(120,20), wxALIGN_CENTRE, _T("ID_STATICTEXT_SH_LD_NAME"));
    shStaticTextLoadName->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_HIGHLIGHTTEXT));
    StaticBitmap1 = new wxStaticBitmap(shPanel1, ID_STATICBITMAP1, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/TMPL8LN.BMP"))), wxPoint(80,288), wxDefaultSize, wxSIMPLE_BORDER, _T("ID_STATICBITMAP1"));
    StaticBitmap2 = new wxStaticBitmap(shPanel1, ID_STATICBITMAP2, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/TMPL8LN.BMP"))), wxPoint(80,336), wxDefaultSize, wxSIMPLE_BORDER, _T("ID_STATICBITMAP2"));
    StaticBitmap3 = new wxStaticBitmap(shPanel1, ID_STATICBITMAP3, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/TMPL8LN.BMP"))), wxPoint(80,392), wxDefaultSize, wxSIMPLE_BORDER, _T("ID_STATICBITMAP3"));
    StaticBitmap4 = new wxStaticBitmap(shPanel1, ID_STATICBITMAP4, wxBitmap(wxImage(_T("/home/billt/projects/jhu_EN525.743/arduino/SDcard_tmpl8s/TMPL8LN.BMP"))), wxPoint(80,440), wxDefaultSize, wxSIMPLE_BORDER, _T("ID_STATICBITMAP4"));
    BoxSizer1->Add(shPanel1, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    SetSizer(BoxSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    BoxSizer1->Fit(this);
    BoxSizer1->SetSizeHints(this);

    Connect(ID_BITMAPBUTTON_SH_RM_RT,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&wxSmartHomeServerFrame::OnshBMPbtnRoomRightClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSmartHomeServerFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSmartHomeServerFrame::OnAbout);
    //*)


//    SHinitServerNodeInfo();
    SHinitLoadNodeInfo();


}

wxSmartHomeServerFrame::~wxSmartHomeServerFrame()
{
    //(*Destroy(wxSmartHomeServerFrame)
    //*)

    // close Boost-asio serial port

    // close any network sockets
}

void wxSmartHomeServerFrame::OnQuit(wxCommandEvent& event)
{
    // close Boost-asio serial port

    // close any network sockets

    Close();
}

void wxSmartHomeServerFrame::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to wxSmartHomeServer"));
    // SH_SERV_APP_NAME
}

void wxSmartHomeServerFrame::OnshBMPbtnRoomRightClick(wxCommandEvent& event)
{
    wxLogMessage( "Button Room Right" ) ;
}

void wxSmartHomeServerFrame::OnshBMPbtnRoomLeftClick(wxCommandEvent& event)
{
    wxLogMessage( "Button Room Left" ) ;
}

void wxSmartHomeServerFrame::OnshBMPbtnLoadRightClick(wxCommandEvent& event)
{
    wxLogMessage( "Button Load Right" ) ;
}

void wxSmartHomeServerFrame::OnshBMPbtnLoadLeftClick(wxCommandEvent& event)
{
    wxLogMessage( "Button Load Left" ) ;
}


void wxSmartHomeServerFrame::OnshBMPbtnIncIntClick(wxCommandEvent& event)
{
    wxLogMessage( "Button Up Inc Intensity" ) ;

    if(shCurrentLoadNodeInfo.SHthisNodeIsPowered == NO)
    {
        shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = LOAD_INTENSITY_FULL_OFF + 1;
        shCurrentLoadNodeInfo.SHthisNodeIsPowered = YES;
    }
    else
    {
        if(shCurrentLoadNodeInfo.SHthisNodeLevelCurrent >= LOAD_INTENSITY_MAX)
        {
            shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = LOAD_INTENSITY_MAX;
        }
        else
        {
            shCurrentLoadNodeInfo.SHthisNodeLevelCurrent += 1;
        }


        if( shCurrentLoadNodeInfo.SHthisNodeLevelCurrent > LOAD_INTENSITY_FULL_OFF )
        {
            shCurrentLoadNodeInfo.SHthisNodeIsPowered = YES;
        }
    }

    // send new level and powered state to the load via Zigbee


    // update the local level/intensity guage on LCD
    SHguageDrawCurrentIntensity();

}

void wxSmartHomeServerFrame::OnshBMPbtnFavIntClick(wxCommandEvent& event)
{
    wxLogMessage( "Button FAVorite Intensity = %d", shCurrentLoadNodeInfo.SHthisNodeLevelFav ) ;

    shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = shCurrentLoadNodeInfo.SHthisNodeLevelFav;

    if(shCurrentLoadNodeInfo.SHthisNodeIsPowered == NO)
    {
        shCurrentLoadNodeInfo.SHthisNodeIsPowered = YES;
    }

    // send new level and powered state to the load via Zigbee


    // update the local level/intensity guage on LCD
    SHguageDrawCurrentIntensity();
}

void wxSmartHomeServerFrame::OnshBMPbtnDecIntClick(wxCommandEvent& event)
{
    wxLogMessage( "Button Down Dec Intensity" ) ;

    if(shCurrentLoadNodeInfo.SHthisNodeLevelCurrent <= LOAD_INTENSITY_MIN)
    {
        shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = LOAD_INTENSITY_MIN;
    }
    else
    {
        shCurrentLoadNodeInfo.SHthisNodeLevelCurrent -= 1;
    }


    if( shCurrentLoadNodeInfo.SHthisNodeLevelCurrent <= LOAD_INTENSITY_FULL_OFF )
    {
        shCurrentLoadNodeInfo.SHthisNodeIsPowered = NO;
    }

    // send new level and powered state to the load via Zigbee


    // update the local level/intensity guage on LCD
    SHguageDrawCurrentIntensity();
}


// Have an ON button event
void wxSmartHomeServerFrame::OnshBMPbtnOnClick(wxCommandEvent& event)
{
    wxLogMessage( "Button On" ) ;

    shCurrentLoadNodeInfo.SHthisNodeIsPowered = YES;

    // send new level and powered state to the load via Zigbee


    // update the local level/intensity guage on LCD
    SHguageDrawCurrentIntensity();


}


// Have an OFF button event
void wxSmartHomeServerFrame::OnshBMPbtnOffClick(wxCommandEvent& event)
{
    wxLogMessage( "Button Off" ) ;

    shCurrentLoadNodeInfo.SHthisNodeIsPowered = NO;

    // send new level and powered state to the load via Zigbee


    // update the local level/intensity guage on LCD
    SHguageDrawCurrentIntensity();
}


// Update the intensity/level guage drawn to new current intensity level
void wxSmartHomeServerFrame::SHguageDrawCurrentIntensity(void)
{
    if( shCurrentLoadNodeInfo.SHthisNodeIsPowered == NO )
    {
        shIntensityGauge->SetValue(0);
    }
    else
    {
        switch(shCurrentLoadNodeInfo.SHthisNodeLevelCurrent)
        {
            case 5:
                shIntensityGauge->SetValue(100);
                break;

            case 4:
                shIntensityGauge->SetValue(80);
                break;

            case 3:
                shIntensityGauge->SetValue(60);
                break;

            case 2:
                shIntensityGauge->SetValue(40);
                break;

            case 1:
                shIntensityGauge->SetValue(20);
                break;

            case 0:
                shIntensityGauge->SetValue(0);
                break;

            default:
                // for any unknown value, reset to 0
                shCurrentIntensity = 0;
                shIntensityGauge->SetValue(0);
                break;
        }
    }
}


#if 1
// Initialize the currentNodeInfo struct based on data in the default load for this server from file
void wxSmartHomeServerFrame::SHinitServerNodeInfo(void)
{
    // files to read data from
//    ifstream FILEserverNodeID;
    FILE  *FILEserverNodeID;

    #define FILENAME_SERVER_NODEINFO  "/home/smarthome/.wxSmartHome/SERVER.BIN"
//    FILEserverNodeID.open(FILENAME_SERVER_NODEINFO, (ios::in | ios::binary) );
    FILEserverNodeID = fopen(FILENAME_SERVER_NODEINFO, FILE_READ);

    ptrThisNodeID[1] = (uint8_t)fgetc(FILEserverNodeID);  // 1st byte
    ptrThisNodeID[0] = (uint8_t)fgetc(FILEserverNodeID);  // 2nd byte

    shThisNodeType = (uint8_t)fgetc(FILEserverNodeID);  // 3rd byte

//    FILEserverNodeID.close();
    fclose(FILEserverNodeID);
}


// read node ID from NodeInfo struct based on data from file
// for the given room number and load number
uint16_t wxSmartHomeServerFrame::SHgetLoadNodeIDfromStorage(uint16_t roomNum, uint8_t loadNum)
{
     uint16_t    tmpNodeID = 0;
     uint8_t    *ptrTmpNodeID = (uint8_t *)&tmpNodeID;

    // files to read data from
//    ifstream FILEloadNodeInfo;
    FILE  *FILEloadNodeInfo;
    char   fileNameLoadData[45];

    sprintf(fileNameLoadData, "/home/smarthome/.wxSmartHome/%d/%d.BIN", roomNum, loadNum);
//    FILEloadNodeInfo.open(fileNameLoadData, (ios::in | ios::binary) );
    FILEloadNodeInfo = fopen(fileNameLoadData, FILE_READ );

    ptrTmpNodeID[1] = (uint8_t)fgetc(FILEloadNodeInfo);  // 1st byte
    ptrTmpNodeID[0] = (uint8_t)fgetc(FILEloadNodeInfo);  // 2nd byte

//    FILEloadNodeInfo.close();
    fclose(FILEloadNodeInfo);

    return(tmpNodeID);
}


// read node type from NodeInfo struct based on data from file
// for the given room number and load number
uint8_t wxSmartHomeServerFrame::SHgetLoadNodeTypefromStorage(uint16_t roomNum, uint8_t loadNum)
{
     uint8_t     tmpNodeType = 0;

    // files to read data from
//    ifstream FILEloadNodeInfo;
    FILE  *FILEloadNodeInfo;
    char   fileNameLoadData[45];

    sprintf(fileNameLoadData, "/home/smarthome/.wxSmartHome/%d/%d.BIN", roomNum, loadNum);
//    FILEloadNodeInfo.open(fileNameLoadData, (ios::in | ios::binary) );
    FILEloadNodeInfo = fopen(fileNameLoadData, FILE_READ );

    (uint8_t)fgetc(FILEloadNodeInfo);  // 1st byte
    (uint8_t)fgetc(FILEloadNodeInfo);  // 2nd byte

    tmpNodeType = (uint8_t)fgetc(FILEloadNodeInfo);  // 3rd byte

//    FILEloadNodeInfo.close();
    fclose(FILEloadNodeInfo);

    return(tmpNodeType);
}

#endif // 0


void wxSmartHomeServerFrame::SHinitLoadNodeInfo(void)
{
    shCurrentIntensity = LOAD_INTENSITY_FULL_OFF; // 0

    shCurrentLoadNodeInfo.SHthisNodeLoc   = DEFAULT_ROOM_NUM;
    shCurrentLoadNodeInfo.SHthisNodeID    = SHgetLoadNodeIDfromStorage(shCurrentLoadNodeInfo.SHthisNodeLoc, DEFAULT_LOAD_NUM);
    shCurrentLoadNodeInfo.SHthisNodeType  = SHgetLoadNodeTypefromStorage(shCurrentLoadNodeInfo.SHthisNodeLoc, DEFAULT_LOAD_NUM);
    shCurrentLoadNodeInfo.SHthisNodePin   = 0; // pin is an Arduino thing, not used in Server code

    shCurrentLoadNodeInfo.SHothrNodeID = shThisNodeID;  // the other node iD talking to a Load is this server
    shCurrentLoadNodeInfo.SHmsgCurrentState = SH_MSG_ST_IDLE;
    shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
    shCurrentLoadNodeInfo.SHmsgCmd = SH_CMD_NOP;
    shCurrentLoadNodeInfo.SHmsgStatus = 0;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHothrID = shThisNodeID;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_ST_IDLE;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_NOP;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusH = 0;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusL = 0;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;// 16bit alternative to SHstatusH and SHstatusL but represents same bytes in message
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal = 0;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHreserved1 = 'R';
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHreserved2 = 'r';
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
    shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;
    shCurrentLoadNodeInfo.newSHmsgTX = NO;
    shCurrentLoadNodeInfo.newSHmsgRX = NO;

    // ask via Zigbee what these values are
    shCurrentLoadNodeInfo.SHthisNodeIsPowered = NO;
    shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = 0;
    shCurrentLoadNodeInfo.SHthisNodeLevelFav = 3;

    wxLogMessage( "Default Load ID = 0x%x", shCurrentLoadNodeInfo.SHthisNodeID ) ;

}
