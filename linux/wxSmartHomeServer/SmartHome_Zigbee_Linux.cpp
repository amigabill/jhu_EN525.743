/***************************************************************
 * Name:      SmartHome_Zigbee_Linux.cpp
 * Purpose:   Implements Zigbee class member functions Linux "gateway" node
 *            to send and receive SmartHome messages over Zigbee network
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-17
 * Copyright: Bill Toner (2015)
 * License:   This library is free software; you can redistribute it and/or
 *            modify it under the terms of the GNU Lesser General Public
 *            License as published by the Free Software Foundation; either
 *            version 2.1 of the License, or (at your option) any later version.
 *            http://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
 * **************************************************************/

// The major difference between this file and the Arduino equivalent
// is how to access serial port data. Arduino uses Serial class,
// Linux uses File Descriptor functions.
// Then the Arduino equivalent uses Arduino Serial.print() calls for debug information,
// and there is no directly comparable thing in Linux method, I'd probably use
// some wxWidgets dialog window or printf to STDOUT/STDERR.
// Normal runtime function should not be different to the outside observer.
// Hopefully I can figure out how to share the same file between them at some point.

// define on compiler commandline instead - #define SERIALPORT_LINUX

//#define DEBUG_ZB_RECEIVE


#include <wx/log.h>


#include "SmartHomeServerAppDetails.h"

#include "SmartHome_Zigbee_Linux.h"
#include "shLinuxSerialPort.h"


// Comment out the following defines to save Arduino resources
// UNcomment them to activate the related debug output
//#define DEBUG_ZB_XMIT
//#define DEBUG_ZB_RECEIVE


// constructor
SHzigbee::SHzigbee(void)
{
	_initXmitAPIframe();
    //initSHmsgRX();

    _ZBoffsetRXbuff = ZB_FRM_OFFSET_DELMTR; //ZB_START_DELIMITER; //Delimiter byte is offset 0 into RX buffer

	ZBnewFrameRXed = NO;
	newSHmsgRX = NO;
	ZBinFrameRX = NO;
	newSHmsgTX = NO;

	SHcmdEventNeedsLogged = NO;
}


#if 1
SHzigbee::~SHzigbee(void)
{
    shSerialPortZigbee.stop();
}
#endif // 0


// set the serial port baud rate used for Zigbee communications
bool SHzigbee::start(const char* portName, unsigned int baud_rate)
{
    shSerialPortZigbee.start(portName, baud_rate);
}


// Initialize the TX API frame with values we will use as standard
void SHzigbee::_initXmitAPIframe(void)
{
    myZBframeTX.ZBfrmDelimiter  = ZB_START_DELIMITER; // 0x7e;
    myZBframeTX.ZBfrmLength     = ZB_TX_FRM_LEN;
    myZBframeTX.ZBfrmType       = ZB_FRAME_TYPE_TX_REQ;
    myZBframeTX.ZBfrmID         = (uint8_t)1; // always use Zigbee Frame ID of 1
    myZBframeTX.ZBdaddr64High   = ZB_64ADDR_BCAST_HIGH;
    myZBframeTX.ZBdaddr64Low    = ZB_64ADDR_BCAST_LOW;
    myZBframeTX.ZBdaddr16       = ZB_16ADDR_BCAST;
    myZBframeTX.ZBfrmRadius     = ZB_BCAST_RADIUS;
    myZBframeTX.ZBfrmOptions    = ZB_OPTIONS;
}


// Transmit a Zigbee API TX Request Frame
// Each Zigbee message frame has a 12byte payload, the SmartHome Message,
// so length=23=0x17 bytes, total frame=27bytes (<- double check that)
// Zigbee data is defined to be Big-Endian (BE) while Arduino AVR is Little-Endian (LE)
// so will need to take care to assemble 16bit and 32bit ints correctly
uint8_t SHzigbee::zbXmitAPIframe(void)
{
    // use some pointer to unsigned int 8 (byte) to access parts of larger datatypes a byte at a time during Zigbee frame transmit
    uint8_t *ptrZBfrmLength           = (uint8_t *)&(myZBframeTX.ZBfrmLength);
    uint8_t *ptrZBdaddr64H            = (uint8_t *)&(myZBframeTX.ZBdaddr64High);
    uint8_t *ptrZBdaddr64L            = (uint8_t *)&(myZBframeTX.ZBdaddr64Low);
    uint8_t *ptrZBfrmPldSHdestAddr16  = (uint8_t *)&(myZBframeTX.ZBfrmPayload.SHdestID);
    uint8_t *ptrZBfrmPldSHsrcAddr16   = (uint8_t *)&(myZBframeTX.ZBfrmPayload.SHsrcID);
    uint8_t *ptrSHdestAddr16          = (uint8_t *)&(SHmsgTX.SHdestID); // Pointer into SHmsgRX received SmartHome message struct
    uint8_t *ptrSHsrcAddr16           = (uint8_t *)&(SHmsgTX.SHsrcID);  // Pointer into SHmsgRX received SmartHome message struct


    // initialize the Zigbee TX API frame checksum before calculating it
    myZBframeTX.ZBfrmChksum = 0;

    // Get data out of our Zigbee frame struct and into a buffer array of bytes/uint8_t
    // for Serial write call to Zigbee module

    _ZBfrmBufferTX[ZB_FRM_OFFSET_DELMTR]       = myZBframeTX.ZBfrmDelimiter; //(uint8_t)0x7e;

    _ZBfrmBufferTX[ZB_FRM_OFFSET_LENH]         = ptrZBfrmLength[1]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_LENL]         = ptrZBfrmLength[0]; //(uint8_t)0x1a;

    // START calculating ZB frame checksum, also the ZB "Frame Length" begins here

    _ZBfrmBufferTX[ZB_FRM_OFFSET_FTYPE]        = myZBframeTX.ZBfrmType; //(uint8_t)0x10;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmType);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_FID]          = myZBframeTX.ZBfrmID; //(uint8_t)0x01;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmID);

    // 64bit Zigbee addr
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B7] = ptrZBdaddr64H[3]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B6] = ptrZBdaddr64H[2]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B5] = ptrZBdaddr64H[1]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B4] = ptrZBdaddr64H[0]; //(uint8_t)0x00;
    myZBframeTX.ZBfrmChksum += _calcChkSum32(myZBframeTX.ZBdaddr64High);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B3] = ptrZBdaddr64L[3]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B2] = ptrZBdaddr64L[2]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B1] = ptrZBdaddr64L[1]; //(uint8_t)0xff;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B0] = ptrZBdaddr64L[0]; //(uint8_t)0xff;
    myZBframeTX.ZBfrmChksum += _calcChkSum32(myZBframeTX.ZBdaddr64Low);

    // 16bit Zigbee addr
    uint8_t *ptrZBdaddr16 = (uint8_t *)&(myZBframeTX.ZBdaddr16);
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR16H] = ptrZBdaddr16[1]; //(uint8_t)0xff;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR16L] = ptrZBdaddr16[0]; //(uint8_t)0xfe;
    myZBframeTX.ZBfrmChksum += _calcChkSum16(myZBframeTX.ZBdaddr16);

    // Zigbee Radius and Options bytes
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_BRADIUS] = myZBframeTX.ZBfrmRadius; //(uint8_t)0x00;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmRadius);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_OPTIONS] = myZBframeTX.ZBfrmOptions; //(uint8_t)0x00;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmOptions);


    // SmartHome message payload bytes
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H]  = ptrZBfrmPldSHdestAddr16[1]; //(uint8_t)0xab;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L]  = ptrZBfrmPldSHdestAddr16[0]; //(uint8_t)0xcd;
    myZBframeTX.ZBfrmChksum += _calcChkSum16(myZBframeTX.ZBfrmPayload.SHdestID);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H]   = ptrZBfrmPldSHsrcAddr16[1]; //(uint8_t)0xf0;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L]   = ptrZBfrmPldSHsrcAddr16[0]; //(uint8_t)0x0d;
    myZBframeTX.ZBfrmChksum += _calcChkSum16(myZBframeTX.ZBfrmPayload.SHsrcID);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE]   = myZBframeTX.ZBfrmPayload.SHmsgType; //(uint8_t)0x01;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHmsgType);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_CMD]        = myZBframeTX.ZBfrmPayload.SHcommand; //(uint8_t)0x01;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHcommand);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_H]   = myZBframeTX.ZBfrmPayload.SHstatusH; //(uint8_t)0x00;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusH);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_L]   = myZBframeTX.ZBfrmPayload.SHstatusL; //(uint8_t)0x00;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusL);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL] = myZBframeTX.ZBfrmPayload.SHstatusVal; //(uint8_t)0x00;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusVal);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_RESVD_1]    = myZBframeTX.ZBfrmPayload.SHreserved1; //(uint8_t)0x54;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved1);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_RESVD_2]    = myZBframeTX.ZBfrmPayload.SHreserved2; //(uint8_t)0x58;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved2);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_CHKSUM]     = myZBframeTX.ZBfrmPayload.SHpayldChksum; //(uint8_t)0xdc;
    myZBframeTX.ZBfrmChksum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHpayldChksum);


    // STOP calculating ZB frame checksum, also ZB "Frame Length" ends here
    myZBframeTX.ZBfrmChksum= (uint8_t)0xff -  myZBframeTX.ZBfrmChksum;

    // Zigbee Frame Checksum
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_CHKSUM] = myZBframeTX.ZBfrmChksum; //(uint8_t)0xf4; //myZBframeTX.ZBfrmChksum;


#ifdef SERIALPORT_ARDUINO
//    Serial.println("");

//    Serial.write(_ZBfrmBufferTX, ZB_TX_FRM_BYTES);
//    Serial.println("Finished sending TXframe");
#endif // ARDUINO
#ifdef SERIALPORT_LINUX
    int bytes_written = 0;
//    bytes_written = write(_shSerialPortZigbee._shSerialPortFD, _ZBfrmBufferTX, ZB_TX_FRM_BYTES);
    _debugPrintZBframeBufTX();
    bytes_written = write(shSerialPortZigbee.shSerialPortFD, _ZBfrmBufferTX, ZB_TX_FRM_BYTES);
#endif // LINUX

}


// debug output the Transmit buffer to a popup window
void SHzigbee::_debugPrintZBframeBufTX(void)
{
    int i=0;
    wxString wxZBbuffTX;


    for(i=0; i<ZB_TX_FRM_BYTES; i++)
    {
        wxZBbuffTX = wxZBbuffTX + wxString::Format("%.2x ", _ZBfrmBufferTX[i]);
    }

//    wxZBbuffTX = "Transmitting Zigbee frame  ";
//    wxLogMessage( wxZBbuffTX ) ;
}


// debug output the Receive buffer to a popup window
void SHzigbee::_debugPrintZBframeBufRX(void)
{
    int i=0;
    wxString wxZBbuffRX;

    wxZBbuffRX = "Received Zigbee frame  ";

    for(i=0; i<ZB_RX_FRM_BYTES; i++)
    {
        wxZBbuffRX = wxZBbuffRX + wxString::Format("%.2x ", _ZBfrmBufferRX[i]);
    }

     wxLogMessage( wxZBbuffRX ) ;
}


// calculate and return 8bit checksum for given 8bit value
uint8_t SHzigbee::_calcChkSum8(uint8_t ui8)
{
//    Serial.print(ui8, HEX);
//    Serial.print(" ");
    return(ui8);
}

// calculate and return 16bit checksum for given 8bit value
uint8_t SHzigbee::_calcChkSum16(uint16_t ui16)
{
    uint8_t *ptrUI8AsUi16 = (uint8_t *)&ui16;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi16[1];
//    Serial.print(ptrUI8AsUi16[1], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi16[0];
//    Serial.print(ptrUI8AsUi16[0], HEX);
//    Serial.print(" ");

    return(tmpChkSum);
}

// calculate and return 32bit checksum for given 8bit value
uint8_t SHzigbee::_calcChkSum32(uint32_t ui32)
{
    uint8_t *ptrUI8AsUi32 = (uint8_t *)&ui32;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi32[3];
//    Serial.print(ptrUI8AsUi32[3], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[2];
//    Serial.print(ptrUI8AsUi32[2], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[1];
//    Serial.print(ptrUI8AsUi32[1], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[0];
//    Serial.print(ptrUI8AsUi32[0], HEX);
//    Serial.print(" ");

    return(tmpChkSum);
}


// Prepare the TX frame message payload to be sent
void SHzigbee::prepareTXmsg( uint16_t prepSHdestID,     // Dest ID
                             uint16_t prepSHsrcID,      // Source ID
                             uint8_t  prepSHmsgType,    // Msg Type
                             uint8_t  prepSHcommand,    // CMD
                             uint8_t  prepSHstatusH,    // Status/StatusID High byte
                             uint8_t  prepSHstatusL,    // Status/StatusID Low byte
                             uint8_t  prepSHstatusVal   // Status value (8bit)
                           )
{
    uint8_t tmpChkSum = 0;

    // clear the checksum value from previous message
    myZBframeTX.ZBfrmPayload.SHpayldChksum= 0;

    // ints are 16bit Little Endian, longs are 32bit Little Endian
    // Zigbee goes Big Endian
    myZBframeTX.ZBfrmPayload.SHdestID    = prepSHdestID;
    tmpChkSum += _calcChkSum16(myZBframeTX.ZBfrmPayload.SHdestID);

    myZBframeTX.ZBfrmPayload.SHsrcID     = prepSHsrcID;
    tmpChkSum += _calcChkSum16(myZBframeTX.ZBfrmPayload.SHsrcID);

    myZBframeTX.ZBfrmPayload.SHmsgType   = prepSHmsgType;
    tmpChkSum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHmsgType);

    myZBframeTX.ZBfrmPayload.SHcommand   = prepSHcommand;
    tmpChkSum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHcommand);

    myZBframeTX.ZBfrmPayload.SHstatusH   = prepSHstatusH;
    tmpChkSum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusH);

    myZBframeTX.ZBfrmPayload.SHstatusL   = prepSHstatusL;
    tmpChkSum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusL);

    myZBframeTX.ZBfrmPayload.SHstatusVal = prepSHstatusVal;
    tmpChkSum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusVal);

    myZBframeTX.ZBfrmPayload.SHreserved1 = (uint8_t)'T';
    tmpChkSum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved1);

    myZBframeTX.ZBfrmPayload.SHreserved2 = (uint8_t)'X';
    tmpChkSum += _calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved2);

    // finish calc payload message checksum
    tmpChkSum = (uint8_t)0xff - tmpChkSum;
    myZBframeTX.ZBfrmPayload.SHpayldChksum  = tmpChkSum;
}


// return the SmartHome message type for this Zigbee frame
uint8_t SHzigbee::getMsgTypeTX(void)
{
    return( myZBframeTX.ZBfrmPayload.SHmsgType);
}


// Receive (attempt to) a Zigbee API RX Received Frame
// Each Zigbee RX RCVD message frame has a 12byte payload, the SmartHome Message,
// so length=24=0x18 bytes, total frame=28bytes (<- double check that)
// Zigbee data is defined to be Big-Endian (BE) while Arduino AVR is Little-Endian (LE)
// so will need to take care to assemble 16bit and 32bit ints correctly
//
// NOTE: Prevoius attempts to receive wre flaky and dropped characters expected inthe message, spradically.
// Now, this works better, to get the received bytes into a buffer ASAP, then later go through the buffer
// and see what we get there for the other data structures to store.
// The previous attempt was to try and get each byte into data structures, update running checksum, etc. per byte received,
// and now that seems to maybe have been too much to do between bytes during the serial transfer, as I'm guessing that
// the Zbee RX buffer became full and bytes were tossed out if nto enough room for them, if this was not reading them fast enough.
uint8_t SHzigbee::zbRcvAPIframe(void)
{
    uint8_t ZB_frm_byte[1]; // byte received in uart RX by Serial.read
    int bytes_read = 0;

    uint16_t ZBfrmLen16bit = 0;
    uint8_t  *ptrFrmLen16bit = (uint8_t *)&ZBfrmLen16bit;
    uint8_t  *ptrZBbufFrmType = _ZBfrmBufferRX + ZB_FRM_OFFSET_FTYPE;


    // Check if a new ZB frame is incoming

    // Read a byte from uart RX buffer
    bytes_read = read(shSerialPortZigbee.shSerialPortFD, ZB_frm_byte, 1);

	if(bytes_read > 0)
    {
        // check if starting a new frame
        if ((NO == ZBinFrameRX) && ( ZB_START_DELIMITER == ZB_frm_byte[0] ))
    	{
            // beginning a new frame
            ZBinFrameRX = YES;
            _ZBoffsetRXbuff = 0; //ZB_START_DELIMITER; //Delimiter byte is offset 0 into RX buffer

            _ZBfrmBufferRX[ZB_FRM_OFFSET_DELMTR] = ZB_frm_byte[0];

            // wait for next byte
            while( read(shSerialPortZigbee.shSerialPortFD, ZB_frm_byte, 1) == 0);

            _ZBfrmBufferRX[ZB_FRM_OFFSET_LENH] = ZB_frm_byte[0];

            // wait for next byte
            while( read(shSerialPortZigbee.shSerialPortFD, ZB_frm_byte, 1) == 0);

            _ZBfrmBufferRX[ZB_FRM_OFFSET_LENL] = ZB_frm_byte[0];


            ptrFrmLen16bit[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENH];
            ptrFrmLen16bit[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENL];

	        // check Z frame length and ignore frames not matching our SmartHome message frame length
	        if(ZB_RX_FRM_LEN == ZBfrmLen16bit)
	        {
                int zbRXbytes = ZB_RX_FRM_LEN; //to see value of ZB_RX_FRM_LENin debugger, no other use

	            // read remainder of frame into buffer, including the checksum which is not included in frame length size
                int i=0;
                while(i < (ZBfrmLen16bit+1) )  // +1 to get the ZB frame checksum byte which is excluded from the provided frame length
                {
                    do {
                        bytes_read = read( shSerialPortZigbee.shSerialPortFD, ZB_frm_byte, 1 );
                        _ZBfrmBufferRX[ZB_FRM_OFFSET_FTYPE + i] = ZB_frm_byte[0];
                    } while(bytes_read <= 0);

                    i += bytes_read;
                }
//                #ifdef DEBUG_ZB_RECEIVE
	            // debug put frame buffer content to serial monitor for viewing
//	            _debugPrintZBframeBufRX();
//                #endif // DEBUG_ZB_RECEIVE

	            // pull data fields out of RX buffer into myZBframeRX and SHmsgRX data structures
	            _parseZBbufferRX();

    		    // indicate to other code that a new SmartHome message has been received for processing
		        newSHmsgRX = YES;

                if(SH_MSG_TYPE_COMPLETED == myZBframeRX.ZBfrmPayload.SHmsgType)
                //if(1)
                {
                    //Server logs ALL completed messages, regardless of control/load IDs involved
                    SHcmdEventNeedsLogged= YES;
                }
	        }

	        // indicate no longer still receiving a ZB frame
            ZBinFrameRX = NO;
	    }
    }
}


// Copy the various data fields from the RX Zigbee frame buffer into a struct for use
void SHzigbee::_parseZBbufferRX(void)
{
    // use some pointer to unsigned int 8 (byte) to access parts of larger datatypes a byte at a time during Zigbee frame receive
    uint8_t *ptrZBfrmLength           = (uint8_t *)&(myZBframeRX.ZBfrmLength);
    uint8_t *ptrZBsaddr64H            = (uint8_t *)&(myZBframeRX.ZBsaddr64High);
    uint8_t *ptrZBsaddr64L            = (uint8_t *)&(myZBframeRX.ZBsaddr64Low);
    uint8_t *ptrZBsaddr16             = (uint8_t *)&(myZBframeRX.ZBsaddr16);
    uint8_t *ptrZBfrmPldSHdestAddr16  = (uint8_t *)&(myZBframeRX.ZBfrmPayload.SHdestID);
    uint8_t *ptrZBfrmPldSHsrcAddr16   = (uint8_t *)&(myZBframeRX.ZBfrmPayload.SHsrcID);
    uint8_t *ptrSHdestAddr16          = (uint8_t *)&(SHmsgRX.SHdestID); // Pointer into SHmsgRX received SmartHome message struct
    uint8_t *ptrSHsrcAddr16           = (uint8_t *)&(SHmsgRX.SHsrcID);  // Pointer into SHmsgRX received SmartHome message struct
//    uint8_t ZBchksumFromSender = 0; // checksum sent to us for comparison

    myZBframeRX.ZBfrmChksum = 0; //new frame starts new checksum
    _SHmessageChksumCalc = 0;

    // Zigbee Frame Delimiter
    myZBframeRX.ZBfrmDelimiter = _ZBfrmBufferRX[ZB_FRM_OFFSET_DELMTR];  // ZB Frame Delimiter

    // Zigbee 16bit BE Frame Length
    ptrZBfrmLength[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENH];  //ZB frame length BE MSbyte
    ptrZBfrmLength[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENL];  //ZB frame length BE LSbyte

    // start calculating RX frame checksum for comparison
    // checksum is calculated on bytes BETWEEN (not including) the Zigbee frame length and checksum byte offsets

    // Zigbee Frame Type
    myZBframeRX.ZBfrmType = _ZBfrmBufferRX[ZB_FRM_OFFSET_FTYPE];  //ZB frame Type
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_FTYPE];

    // Zigbee 64bit BE Source addr
    ptrZBsaddr64H[3] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B7];  // ZB frame BE 64bit DADDR Hword MSbyte 7
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B7];

    ptrZBsaddr64H[2] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B6];  // ZB frame BE 64bit DADDR Hword byte 6
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B6];

    ptrZBsaddr64H[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B5];  // ZB frame BE 64bit DADDR Hword byte 5
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B5];

    ptrZBsaddr64H[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B4];  // ZB frame BE 64bit DADDR Hword LSbyte 4
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B4];

    ptrZBsaddr64L[3] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B3];  // ZB frame BE 64bit DADDR Lword MSbyte 3
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B3];

    ptrZBsaddr64L[2] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B2];  // ZB frame BE 64bit DADDR Lword byte 2
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B2];

    ptrZBsaddr64L[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B1];  // ZB frame BE 64bit DADDR Lword byte 1
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B1];

    ptrZBsaddr64L[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B0];  // ZB frame BE 64bit DADDR Lword LSbyte 0
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B0];

    // Zigbee 16bit BE Source addr
    ptrZBsaddr16[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16H];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16H];

    ptrZBsaddr16[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16L];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16L];

    // Zigbee Options byte
    myZBframeRX.ZBfrmOptions = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_OPTIONS];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_OPTIONS];

    //// BEGIN SmartHome message payload bytes

    //// ZB Payload - SmartHome 16bit BE Node Destination Address
    ptrZBfrmPldSHdestAddr16[1] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H)];

    ptrZBfrmPldSHdestAddr16[0] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L)];

    SHmsgRX.SHdestID = myZBframeRX.ZBfrmPayload.SHdestID;

    //// ZB Payload - SmartHome 16bit BE Node Source Address
    ptrZBfrmPldSHsrcAddr16[1] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H)];

    ptrZBfrmPldSHsrcAddr16[0] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L)];

    SHmsgRX.SHsrcID = myZBframeRX.ZBfrmPayload.SHsrcID;

    //// ZB Payload - SmartHome Message Type (CMD_INIT, ACK_REQ, CONFIRM, COMPLETE)
    myZBframeRX.ZBfrmPayload.SHmsgType = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE)];

    SHmsgRX.SHmsgType = myZBframeRX.ZBfrmPayload.SHmsgType;

    //// ZB Payload - SmartHome Command
    myZBframeRX.ZBfrmPayload.SHcommand = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CMD)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CMD)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CMD)];

    SHmsgRX.SHcommand = myZBframeRX.ZBfrmPayload.SHcommand;

    //// ZB Payload - SmartHome Status 2=H / 16bit StatusID BE Hbyte/MSbyte
    myZBframeRX.ZBfrmPayload.SHstatusH = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_H)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_H)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_H)];

    SHmsgRX.SHstatusH = myZBframeRX.ZBfrmPayload.SHstatusH;

    //// ZB Payload - SmartHome Status 1=L / 16bit StatusID BE Lbyte/LSbyte
    myZBframeRX.ZBfrmPayload.SHstatusL = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_L)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_L)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_L)];

    SHmsgRX.SHstatusL = myZBframeRX.ZBfrmPayload.SHstatusL;

    //// ZB Payload - SmartHome Status 0=StatusVal 8bits
    myZBframeRX.ZBfrmPayload.SHstatusVal = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL)];

    SHmsgRX.SHstatusVal = myZBframeRX.ZBfrmPayload.SHstatusVal;

    //// ZB Payload - SmartHome Reserved Bytes (for Future Use)
    myZBframeRX.ZBfrmPayload.SHreserved1 = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_1)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_1)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_1)];

    SHmsgRX.SHreserved1 = myZBframeRX.ZBfrmPayload.SHreserved1;

    myZBframeRX.ZBfrmPayload.SHreserved2 = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_2)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_2)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_2)];

    SHmsgRX.SHreserved2 = myZBframeRX.ZBfrmPayload.SHreserved2;

    //// ZB Payload - SmartHome Message Checksum (provided by the sender)
    myZBframeRX.ZBfrmPayload.SHpayldChksum = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CHKSUM)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CHKSUM)];
    _SHmessageChksumCalc = (uint8_t)0xff - _SHmessageChksumCalc;

    SHmsgRX.SHpayldChksum = myZBframeRX.ZBfrmPayload.SHpayldChksum;

    //// END of the SmartHome Message / Zigbee Frame Payload

    // END of Zigbee Frame bytes which are included inteh ZB Frame Checksum calculation

    // Zigbee Frame checksum
    myZBframeRX.ZBfrmChksum = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_CHKSUM];
    _ZBfrmRXchkSumCalc = (uint8_t)0xff - _ZBfrmRXchkSumCalc;

    // Debug output to serial monitor
    _debugPrintZBframeStructRX();
    _debugPrintSHmsgRX();
}


// Send the Zigbee frame receive buffer to Serial port monitor for developer/debugging purposes
//void SHzigbee::_debugPrintZBframeBufRX(void)
//{
#ifdef DEBUG_ZB_RECEIVE
    // Arduino code
    uint16_t i=0;

    Serial.print("RX ZF Frame = ");
    for(i=0; i<ZB_RX_FRM_BYTES; i++)
    {
        Serial.print( _ZBfrmBufferRX[i], HEX );
	Serial.print(" ");
    }
    Serial.println("  <ZBbufRX>");
#endif // DEBUG_ZB_RECEIVE
//}


// Send the Zigbee frame receive struct to Serial port monitor for developer/debugging purposes
void SHzigbee::_debugPrintZBframeStructRX(void)
{
#ifdef DEBUG_ZB_RECEIVE
    // Arduino code

//    Serial.print("myZBframeRX.ZBfrmDelimiter (hex) = ");
    Serial.print(myZBframeRX.ZBfrmDelimiter, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmLength (hex) = ");
    Serial.print(myZBframeRX.ZBfrmLength, HEX);
//    Serial.print(" = (decimal) ");
    Serial.print("=");
    Serial.print(myZBframeRX.ZBfrmLength, DEC);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmType (hex) = ");
    Serial.print(myZBframeRX.ZBfrmType, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBsaddr64High (hex) = ");
    Serial.print(myZBframeRX.ZBsaddr64High, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBsaddr64Low (hex) = ");
    Serial.print(myZBframeRX.ZBsaddr64Low, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBsaddr16 (hex) = ");
    Serial.print(myZBframeRX.ZBsaddr16, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmOptions (hex) = ");
    Serial.print(myZBframeRX.ZBfrmOptions, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHdestID (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHdestID, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHsrcID (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHsrcID, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHmsgType (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHmsgType, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHcommand (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHcommand, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHstatusH (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHstatusH, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHstatusL (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHstatusL, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHstatusVal (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHstatusVal, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHreserved1 (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHreserved1, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHreserved2 (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHreserved2, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmPayload.SHpayldChksum (hex) = ");
    Serial.print(myZBframeRX.ZBfrmPayload.SHpayldChksum, HEX);
//    Serial.print(" ?=? _SHmessageChksumCalc =  ");
    Serial.print("?=?");
    Serial.print(_SHmessageChksumCalc, HEX);
    Serial.print(" ");

//    Serial.print("myZBframeRX.ZBfrmChksum (hex) = ");
    Serial.print(myZBframeRX.ZBfrmChksum, HEX);
//    Serial.print(" ?=? _ZBfrmRXchkSumCalc =  ");
    Serial.print("?=?");
    Serial.print(_ZBfrmRXchkSumCalc, HEX);
    Serial.println(" <ZBstructRX>");
#endif // DEBUG_ZB_RECEIVE
}


void SHzigbee::_debugPrintSHmsgRX(void)
{
#ifdef DEBUG_ZB_RECEIVE
    // Arduino code

//    Serial.print("SHmsgRX.SHdestID (hex) = ");
    Serial.print(SHmsgRX.SHdestID, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHsrcID (hex) = ");
    Serial.print(SHmsgRX.SHsrcID, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHmsgType (hex) = ");
    Serial.print(SHmsgRX.SHmsgType, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHcommand (hex) = ");
    Serial.print(SHmsgRX.SHcommand, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHstatusH (hex) = ");
    Serial.print(SHmsgRX.SHstatusH, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHstatusL (hex) = ");
    Serial.print(SHmsgRX.SHstatusL, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHstatusVal (hex) = ");
    Serial.print(SHmsgRX.SHstatusVal, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHreserved1 (hex) = ");
    Serial.print(SHmsgRX.SHreserved1, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHreserved2 (hex) = ");
    Serial.print(SHmsgRX.SHreserved2, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHpayldChksum (hex) = ");
    Serial.print(SHmsgRX.SHpayldChksum, HEX);
//    Serial.print(" ?=? _SHmessageChksumCalc =  ");
    Serial.print("?=?");
    Serial.print(_SHmessageChksumCalc, HEX);
    Serial.println(" <SHmsgRX>");

#endif // DEBUG_ZB_RECEIVE
}

