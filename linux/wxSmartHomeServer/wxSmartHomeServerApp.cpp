/***************************************************************
 * Name:      wxSmartHomeServerApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner ()
 * License:
 * Initially generated by wxSmith GUI builder tool
 **************************************************************/


//(*AppHeaders
#include "wxSmartHomeServerApp.h"
#include "wxSmartHomeServerMain.h"
#include <wx/image.h>
//#include <wx/textfile.h>
#include <wx/file.h>
//*)

//#include "shLinuxSerialPort.h
//shSerialPort mySHserverSerial = shSerialPort();

#define SH_CTRL_EVENT_LOG_FILENAME  (wxString)"/home/smarthome/.wxSmartHome/shEvents.log"


IMPLEMENT_APP(wxSmartHomeServerApp);

BEGIN_EVENT_TABLE(wxSmartHomeServerApp, wxApp)
   EVT_IDLE(wxSmartHomeServerApp::OnIdle)
END_EVENT_TABLE()


bool wxSmartHomeServerApp::OnInit()
{
    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
//    	wxSmartHomeServerFrame* Frame = new wxSmartHomeServerFrame(0);
    	Frame = new wxSmartHomeServerFrame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}



// Do something on an Idle event
// This is where our non-GUI "loop" will happen, such as
// to check on serial input form Zigbee, refresh the log file etc.
void wxSmartHomeServerApp::OnIdle(wxIdleEvent &event)
{
//    wxLogMessage( "In wxSmartHomeServerApp::OnIdle" ) ;

    /* ... */

    // check if a new Zigbee frame/SmartHome message has been received an dprocess it

    // check if a new Zigbee frame/SmartHome message is ready to send out

    // check if the SmartHome event log file has been updated,
    // and write new content to LCD display text area
    SHupdateGUIlogText(SH_CTRL_EVENT_LOG_FILENAME);


    if(IsMainLoopRunning())
        event.RequestMore();
}



// load the log file and refresh the SmartHome Server GUI with any new log content
uint8_t wxSmartHomeServerApp::SHupdateGUIlogText(const wxString& shLogFileName)
{
    #define SH_LOG_FILE_MAX_LINE_LENGTH 200

    size_t i=0, shLogFileumLinesCurr=0;
    wxFile shLogFile(shLogFileName, wxFile::read);
    wxUint8 thisChar[1];


//    wxLogMessage( "Entering SHupdateGUIlogText with log filename = %s", shLogFileName ) ;

    if( !shLogFile.IsOpened() )
    {
        // could not open the file for some reason
        return false;
    }


    wxFileOffset shLogFileLength = shLogFile.Length();
    if( shLogFileLength == wxInvalidOffset )
    {
        // file is of some invalid size for some reason
        return false;
    }

    if(shLogFileLength == shLogFileLengthPrev)
    {
        // no change in file size, assume no change to log since last check.
        // This is not a problem, just nothing to do this time around.
        return true;
    }


    // Clear out the GUI log fie text area before refreshing the text content,
    // in order to avoid duplicated sets before the final new line(s) at the bottom
    Frame->TextCtrl1->Clear();


    // Read a character at a time to build up a text line,
    // then send the line of text to the GUI log text area,
    // and check for any additional text lines to display.
    while( (!shLogFile.Eof()) && (shLogFile.Tell() != wxInvalidOffset) )
    {
        // create a new text line string each iteration, initialize full of NULL characters
        wxString shLogFileTextLine( (wxChar)0x00, SH_LOG_FILE_MAX_LINE_LENGTH );

        i=0;
        do
        {
            // read one char from file and add to current text line string
            shLogFile.Read( thisChar, 1 );
            shLogFileTextLine[i] = (wxChar)thisChar[0];

            i++;
        } while( (thisChar[0] != '\n') && (i<SH_LOG_FILE_MAX_LINE_LENGTH) && (!shLogFile.Eof()) && (shLogFile.Tell() != wxInvalidOffset) );
        shLogFileTextLine[i] = (wxChar)0x00; // make sure to add NULL char at end of text line

        Frame->TextCtrl1->AppendText(shLogFileTextLine);
    }

    // save current log file length for comparison next time around
    shLogFileLengthPrev = shLogFileLength;

    // all good at this point
    return true;
}


