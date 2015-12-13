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
#include <wx/utils.h> //for time wxNow
#include <wx/log.h>
//*)

#include "SmartHomeServerAppDetails.h"


IMPLEMENT_APP(wxSmartHomeServerApp);

BEGIN_EVENT_TABLE(wxSmartHomeServerApp, wxApp)
   EVT_IDLE(wxSmartHomeServerApp::OnIdle)
END_EVENT_TABLE()



bool wxSmartHomeServerApp::OnInit()
{
    _appIsReady = NO;

    _mySHzigbee.start(SH_SERVER_SERIAL_PORT_NAME, SH_SERVER_SERIAL_BAUD_RATE);

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

    // start the serial port with SmartHome system Zigbee settings
//	_shSerialPortZigbee.start(9600);


    // for some reason wxLogDebug() causes seg faults
    //wxLogDebug( wxT("Testing wxDebugLog() call\n") );

//    Frame->shDebugLogFile.Open( SH_DEBUG_LOG_FILENAME, wxFile::write);
//    Frame->shDebugLogFile.Write("Testing debug log output\n");

   // update GUI display with current time
    _SHupdateGUItimeText();

//    wxLogMessage( shCurrentDateTime[11] ) ;

    _appIsReady = YES;

    // make sure not to log any phantom events before we are running and receive real events
    _mySHzigbee.SHcmdEventNeedsLogged = NO;

    return wxsOK;

}



// Do something on an Idle event
// This is where our non-GUI "loop" will happen, such as
// to check on serial input form Zigbee, refresh the log file etc.
void wxSmartHomeServerApp::OnIdle(wxIdleEvent &event)
{
    if(_appIsReady == YES)
    {
//        wxLogMessage( "In wxSmartHomeServerApp::OnIdle" ) ;

        /* ... */


        // check if the SmartHome event log file has been updated,
        // and write new content to LCD display text area
        _SHupdateGUIlogText(SH_CTRL_EVENT_LOG_FILENAME);

        // update GUI display with current time
        _SHupdateGUItimeText();


        // check if Zibee message has been received. If is a COMPLETED message then log the SmartHome command event.
//        if(YES == _mySHzigbee.newSHmsgRX)
        if(YES == _mySHzigbee.SHcmdEventNeedsLogged)
        {
            _logSHcmdEventIfCompleted();
        }

        // check if a new Zigbee frame/SmartHome message has been received and process it
        // check if a new Zigbee frame/SmartHome message is ready to send out
        // Check in on SmartHome Zigbee messaging state machine for anything to do (ready to transmit or already received)
//        if( (YES == Frame->shCurrentLoadNodeInfo.newSHmsgTX) || (YES == Frame->shCurrentLoadNodeInfo.newSHmsgRX) )
        if( (YES == Frame->shCurrentLoadNodeInfo.newSHmsgTX) || (YES == _mySHzigbee.newSHmsgRX) )
        {
            _doServerNodeIDmsgSM();
        }

        // try to receive SmartHome message data from Serial port
        if(NO == _mySHzigbee.ZBinFrameRX)
        {
            _mySHzigbee.zbRcvAPIframe();
        }

        if(YES == Frame->shGetNewLoadCurIntensity)
        {
            _shGUIupdateLoadIntensityLevel();
        }
    }

    // make sure to have another idle event to come into here again
    if( IsMainLoopRunning() )
    {
        event.RequestMore();
    }
}

void wxSmartHomeServerApp::_SHnewMsgUpdateLoadNodeInfo(void)
{
    if( (YES == _mySHzigbee.newSHmsgRX) && (_mySHzigbee.SHmsgRX.SHsrcID == Frame->shCurrentLoadNodeInfo.SHthisNodeID) )
    {
        Frame->shCurrentLoadNodeInfo.SHmsgCmd = _mySHzigbee.SHmsgRX.SHcommand;
        Frame->shCurrentLoadNodeInfo.SHmsgNextState = _mySHzigbee.SHmsgRX.SHmsgType;
        Frame->shCurrentLoadNodeInfo.newSHmsgRX = _mySHzigbee.newSHmsgRX;

//        Frame->shCurrentLoadNodeInfo.SHthisNodeLevelFav = _mySHzigbee.SHmsgRX.SHstatusH; //FAV
//        Frame->shCurrentLoadNodeInfo.SHthisNodeIsPowered = _mySHzigbee.SHmsgRX.SHstatusL; //isPowered
//        Frame->shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = _mySHzigbee.SHmsgRX.SHstatusVal; //CurLevel

        Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHcommand = _mySHzigbee.SHmsgRX.SHcommand;
        Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusH   = _mySHzigbee.SHmsgRX.SHstatusH; //FAV
        Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusL   = _mySHzigbee.SHmsgRX.SHstatusL; //isPowered
        Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal = _mySHzigbee.SHmsgRX.SHstatusVal; //CurLevel

    }
}

// communications state machine, to send/receive Zigbee packets/frames
void wxSmartHomeServerApp::_doServerNodeIDmsgSM(void)
{
    volatile uint8_t   SHmsgNextState;         // which of 4 message stages are we in now, or 0=idle?
    volatile uint8_t   SHmsgCmd;
    volatile uint8_t   SHmsgStatus;


    if( (YES == _mySHzigbee.newSHmsgRX) && (_mySHzigbee.SHmsgRX.SHsrcID == Frame->shCurrentLoadNodeInfo.SHthisNodeID) )
    {
        _SHnewMsgUpdateLoadNodeInfo();
    }

    // update to previous iteration's next state
    Frame->shCurrentLoadNodeInfo.SHmsgCurrentState = Frame->shCurrentLoadNodeInfo.SHmsgNextState;

   switch ( Frame->shCurrentLoadNodeInfo.SHmsgCurrentState )
    {
        // Linux Server will receive ALL messages, and log ALL messages. No need to detect errors and specifically tell server of them.
        //for Linux server logging - use dir name matching node ID with files inside to give room location etc. info

        case SH_MSG_ST_IDLE:
            // Doing nothing
            break;

        case SH_MSG_ST_CMD_INIT: // Server TX - send a new SmartHome command message to a load
            if ( SH_MSG_TYPE_CMD_REQ == Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHmsgType )
            {
                // Prepare and Send the appropriate SmartHome command message via Zigbee
                 _mySHzigbee.prepareTXmsg(
                    Frame->shCurrentLoadNodeInfo.SHthisNodeID,              // DestID is the load target that this WC node wants to control
                    Frame->shThisNodeID,                              // Src ID is this node
                    SH_MSG_TYPE_CMD_REQ,                       // MsgType
                    Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHcommand,   // CMD
                    Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusH,   // Status H byte
                    Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusL,   // Status L byte
                    Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal  // Status value
                );

                _mySHzigbee.zbXmitAPIframe();      // Transmit the SmartHome message over Zigbee
                Frame->shCurrentLoadNodeInfo.newSHmsgTX = NO;  // sent it, so make sure we don't send it again

                Frame->shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_ACK_REQ;
//                Frame->shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
//                Frame->shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_COMPLETE;
            }
            break;

        case SH_MSG_ST_ACK_REQ:  // Server RX - Wait for ACK_REQ from the node we are talking to
            Frame->shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_CNFRM;

            _mySHzigbee.newSHmsgRX = NO;
            break;

        case SH_MSG_ST_CNFRM:  // Server TX - tell other node if we initiated command request CMD_REQ to it or not
            Frame->shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_COMPLETE;
            break;

        case SH_MSG_ST_COMPLETE: // Server RX - wait for Load Driver to tell how command execution went

            // if a message from the currently selected Load node ID, then udpate level indicator
            if( YES == Frame->shCurrentLoadNodeInfo.newSHmsgRX ) //received an appropriate confirmation message
            {

//                wxLogMessage( "Command Complete from node 0x%.4x, status = 0x%x", shCurrentLoadNodeInfo.SHthisNodeMsg.SHsrcID, shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal ) ;
//                wxLogMessage( "Command Complete from node 0x%.4x", Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHothrID ) ;

                switch( Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHcommand )
                {
                    // These commands will redraw the wall control Intensity Level Indicator based on value in statusVal field
                    case SH_CMD_LOAD_READCRNT:
                    case SH_CMD_LOAD_GOTOFAV: // implies will be powered, as FAV cannot be full-off level
//                    case SH_CMD_LOAD_SAVEFAV:
                        Frame->shCurrentLoadNodeInfo.SHthisNodeLevelFav = Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusH;
// wrong?                       Frame->shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal;
// correct? but duped below                        Frame->shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal;

                        // NO break here so we also get the Powered and LevelCurrent values below

                    case SH_CMD_LOAD_ON:
                    case SH_CMD_LOAD_OFF:
                    case SH_CMD_LOAD_TOGLPWR:
//                        Frame->shCurrentLoadNodeInfo.SHthisNodeIsPowered = Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusL;
// wrong?                       Frame->shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal;

                        // NO break here so we also get the Powered and LevelCurrent values below

                    case SH_CMD_LOAD_INC:
                    case SH_CMD_LOAD_DEC:

                        Frame->shCurrentLoadNodeInfo.SHthisNodeIsPowered = Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusL;
                        Frame->shCurrentLoadNodeInfo.SHthisNodeLevelCurrent = Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal;

#if 0
                        if(SH_POWERED_ON == Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusL)
                        {
                            // lcd draw current intensity level to the status val returned in this message
                            lcdDrawLvlIndicator(Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHstatusVal);

                        }
                        else
                        {
                            // NOT currently powered, so lcd draw level indicator as lowest level (OFF), regardless of current intensity level reported by the load
                            //lcdDrawLvlIndicator(0);
                            lcdDrawLvlIndicator(LOAD_INTENSITY_FULL_OFF);
                        }
#endif // 0
                        Frame->SHguageDrawCurrentIntensity();

                        break;

                    default:
                        // do nothing for other command types coming back
                        break;
                }

                Frame->shCurrentLoadNodeInfo.newSHmsgRX = NO;

                // reset back to idle state
                Frame->shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
                Frame->shCurrentLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_IDLE;
            } // if

            // do the message log elsewhere, outside of this state machine as messages may not come
            // from the shCurrentNodeInfo's ID, and we want to log completed messages form ALL senders to ALL receivers
            _mySHzigbee.newSHmsgRX = NO;

            break;

        default:
            // do nothing for other command types coming back
            Frame->shCurrentLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
            break;

    } // switch

}


// If have received a SmartHome message via Zigbee, then check what message it was in the protocol "conversation".
// If it is a command completed message, then log this SmartHome command event to text log file and
// add to the GUI display. Server logs ALL completed messages, regardless of control/load IDs involved
bool wxSmartHomeServerApp::_logSHcmdEventIfCompleted(void)
{
    wxFile shLogFile;
    wxString shLogFileNewLine;

    shLogFile.Open(SH_CTRL_EVENT_LOG_FILENAME, wxFile::write_append);

    if( !shLogFile.IsOpened() )
    {
        // could not open the file for some reason
        return false;
    }


    wxFileOffset shLogFileLengthCurr = shLogFile.Length();
    if( shLogFileLengthCurr == wxInvalidOffset )
    {
        // file is of some invalid size for some reason
        return false;
    }

    // append new message to the text log file
    shLogFileNewLine = _shCurrentFullDate + "   " + _shCurrentTime + "    "
                     + wxString::Format("Control=%.4x  ", _mySHzigbee.myZBframeRX.ZBfrmPayload.SHdestID)
                     + wxString::Format("Load=%.4x    ", _mySHzigbee.myZBframeRX.ZBfrmPayload.SHsrcID)
                     //+ wxString::Format("%.2x ", _mySHzigbee.myZBframeRX.ZBfrmPayload.SHmsgType)
                     + "\n        "
                     + wxSHcommandStrings[_mySHzigbee.myZBframeRX.ZBfrmPayload.SHcommand]
                     + "    Powered=" + wxString::Format("%d", _mySHzigbee.myZBframeRX.ZBfrmPayload.SHstatusL)
                     + "  Level/Status=" + wxString::Format("%d", _mySHzigbee.myZBframeRX.ZBfrmPayload.SHstatusVal)
                     + "\n\n";

    shLogFile.Write( shLogFileNewLine, shLogFileNewLine.Length() );

    shLogFile.Close();

    // clear flag so we don't repeat this new log event
    _mySHzigbee.SHcmdEventNeedsLogged = NO;

    // all good at this point
    return true;

}


// load the log file and refresh the SmartHome Server GUI with any new log content
bool wxSmartHomeServerApp::_SHupdateGUIlogText(const wxString& shLogFileName)
{
    #define SH_LOG_FILE_MAX_LINE_LENGTH 200

    size_t i=0, shLogFileNumLinesCurr=0;
    wxFile shLogFile;
    wxUint8 thisChar[1];

    shLogFile.Open(SH_CTRL_EVENT_LOG_FILENAME, wxFile::read);


//    wxLogMessage( "Entering SHupdateGUIlogText with log filename = %s", shLogFileName ) ;

    if( !shLogFile.IsOpened() )
    {
        // could not open the file for some reason
        return false;
    }


    wxFileOffset shLogFileLengthCurr = shLogFile.Length();
    if( shLogFileLengthCurr == wxInvalidOffset )
    {
        // file is of some invalid size for some reason
        return false;
    }

    if(shLogFileLengthCurr == _shLogFileLengthPrev)
    {
        // no change in file size, assume no change to log since last check.
        // This is not a problem, just nothing to do this time around.
        return true;
    }


    // Clear out the GUI log fie text area before refreshing the text content,
    // in order to avoid duplicated sets before the final new line(s) at the bottom
    Frame->shTextCtrlEvtlog->Clear();


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

        Frame->shTextCtrlEvtlog->AppendText(shLogFileTextLine);
    }

    // save current log file length for comparison next time around
    _shLogFileLengthPrev = shLogFileLengthCurr;

    shLogFile.Close();

    // all good at this point
    return true;
}


// Get the current date and time, and update the Linux Server GUI
uint8_t wxSmartHomeServerApp::_SHupdateGUItimeText(void)
{

//    wxLogMessage( "Entering SHupdateGUIlogText with log filename = %s", shLogFileName ) ;

    // get current time into a string
    _shCurrentDateTime = wxNow();
//    wxLogMessage( "Date + time = %s", _shCurrentDateTime ) ;

    _shCurrentDay = _shCurrentDateTime.SubString(0, 2);
//    wxLogMessage( "Day = %s", _shCurrentDay ) ;

    _shCurrentMonth = _shCurrentDateTime.SubString(4, 6);
//    wxLogMessage( "Month = %s", _shCurrentMonth ) ;

    _shCurrentDate = _shCurrentDateTime.SubString(8, 9);
//    wxLogMessage( "Date = %s", _shCurrentDate ) ;

    _shCurrentYear = _shCurrentDateTime.SubString(20, 23);
//    wxLogMessage( "Year = %s", _shCurrentYear ) ;

    _shCurrentDayFullDate = _shCurrentDay + ", " + _shCurrentMonth + " " + _shCurrentDate + ", " + _shCurrentYear;
    _shCurrentFullDate = _shCurrentMonth + " " + _shCurrentDate + ", " + _shCurrentYear;
//    wxLogMessage( "Full Date = %s", _shCurrentFullDate ) ;

    _shCurrentTime = _shCurrentDateTime.SubString(11, 18);
//    wxLogMessage( "Time = %s", _shCurrentTime ) ;


    // Clear out the GUI log fie text area before refreshing the text content,
    // in order to avoid duplicated sets before the final new line(s) at the bottom
    Frame->shTextCtrlCurrentDate->Clear();
    Frame->shTextCtrlCurrentDate->AppendText(_shCurrentDayFullDate);
    Frame->shTextCtrlCurrentTime->Clear();
    Frame->shTextCtrlCurrentTime->AppendText(_shCurrentTime);

      // all good at this point
    return true;
}


// Receive Serial port/Zigbee data
// if the first byte is the Zigbee delimiter, then look for more bytes
// and verify if it looks like a SmartHome Message frame
uint8_t wxSmartHomeServerApp::_SHreceiveZBserialData(void)
{
//    _shSerialPortZigbee.
}

uint8_t wxSmartHomeServerApp::_SHsendZBserialData(void)
{


}


void wxSmartHomeServerApp::_shGUIupdateLoadIntensityLevel(void)
{

    // clear the flag to avoid duplicate triggers into here
    Frame->shGetNewLoadCurIntensity = NO;
}
