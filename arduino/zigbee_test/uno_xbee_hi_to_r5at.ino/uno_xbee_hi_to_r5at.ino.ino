
// uncomment these DEBUG items to enable debug serial prints
//#define DEBUG_ZB_RX
//#define DEBUG_ZB_TX


#include "Arduino.h"

//#include "../byteswap.h"
//extern "C" {
//  #include "byteswap.h"
//}
// following LGPL 2.1+ byteswap macros slightly modified from http://repo-genesis3.cbi.utsa.edu/crossref/ns-sli/usr/include/bits/byteswap.h.html
// These byteswaps are needed, as 16bit ints and 32bit longs in the Zigbee message byte order are byteswapped 
// compared to the Arduino's requirements to conveniently use them as 16bit int or 32bit long values, 
// rather than doing more work to deal with everythign as individual bytes
#define BYTESWAP16(x) \
       (unsigned int)((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

#define BYTESWAP32(x) \
       (unsigned long)((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |               \
                       (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))

#define  NO 0x00
#define YES 0x01


// The 4 types of SmartHome message to be sent over Zigbee
// together these four messages ame up one SmartHome protocol "conversation"
#define SH_MSG_TYPE_CMD_REQ   0x01 // Request recipient to run a new command
#define SH_MSG_TYPE_ACK_CREQ  0x02 // ACKnowledge new command received, Request COnfirmation from Sender
#define SH_MSG_TYPE_CONFIRMED 0x03 // Sender Confirms it did indeed initiate this new command request
#define SH_MSG_TYPE_COMPLETED 0x04 // Recipient indicates command has been executed and completed


// Xbee module for Zigbee must be preconfigured as a Router in API mode and 9600 8n1



// Select the Microcontroller type for this node unit
#define uC_TYPE_UNO  1  // used for Targets
#define uC_TYPE_DUE  2  // used for Wall "switch" controllers
#define uCtype      uC_TYPE_UNO
//#define uCtype    uC_TYPE_DUE

#define SH_NODE_TYPE_TARGET  0x00  // Target, has Triacs/PowerTail, NO LCD
#define SH_NODE_TYPE_CNTRL   0x01  // Wall "switch" controller, has LCD, uSD
#define SH_NODE_TYPE_SERVER  0x02  // Linux Server (NOT an Arduino platform)

// SmartHome node Commands
#define SH_CMD_NOP          0x00        // No OPeration, do nothing
#define SH_CMD_LOAD_ON      0x01        // Turn on the target load at this node
#define SH_CMD_LOAD_OFF     0x02        // Turn off the target load at this node
// ...

// SmartHome node Status Values
#define SH_STATUS_SUCCESS 0x00
#define SH_STSTUS_FAILED  0x01
#define SH_STATUS_CONFIRMED 0x02
#define SH_STATUS_NOT_CONFIRMED 0x03

#define SH_NODE_ID_NUM_BYTES 2
//#define SH_RESERVED_BYTE 0x00
#define SH_RESERVED_BYTE 'R'

// Smarthome message payload variables
byte SH_TargetLoadID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // destination addr
//byte SH_DestID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // destination node ID
//byte SH_SourceID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // source node ID
byte SH_DestID[SH_NODE_ID_NUM_BYTES] = {'d', 'u'}; // destination node ID
byte SH_SourceID[SH_NODE_ID_NUM_BYTES] = {'d', 'e'}; // source node ID
byte SH_StatusID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // node addr for Status
byte SH_command = SH_CMD_NOP;
byte SH_nodeStatus = 0x00;
byte SH_msgCRC = 0x00;

// Temporary Node ID method, until they can be read from microSD, EEPROM etc.
// Eventually, Unos will read from EEPROM and Dues will read from SD.

#define IDnode      0x01
#define IDloadLight 0x01
#define IDloadFan   0x02

#define ZB_ID_COORD 0x00        // The Zigbee coordinator is always ID 0
#define ZB_ID_PAN   0x42        // The ID for the local network. Other networks are on different PAN IDs.
//#define ZB_ID_H     (byte){0x00, 0x13, 0xa2, 0x00}  // The high part of Zigbee ID for Digi Xbee modules is fixed at 0x0013a200
//byte zb_id_L[] = {0x40, 0xe6, 0xec, 0x86};        // 0x40e6ec86

// Zigbee API frame stuff
#define ZB_START_DELIM               (byte)0x7e // indicates start of a Zigbee frame

#define ZB_FRAME_TYPE_ATCMD_IMD      (byte)0x08 // Transmit Request
#define ZB_FRAME_TYPE_ATCMD_QUE      (byte)0x09 // Transmit Request
#define ZB_FRAME_TYPE_REMCMD_REQ     (byte)0x17 // Transmit Request
#define ZB_FRAME_TYPE_ATCMD_RESP     (byte)0x88 // Transmit Request
#define ZB_FRAME_TYPE_MDM_ST         (byte)0x8a // Modem Status
#define ZB_FRAME_TYPE_TX_REQ         (byte)0x10 // Transmit Request (send a payload)
#define ZB_FRAME_TYPE_TX_RESP        (byte)0x8b // Transmit Response
#define ZB_FRAME_TYPE_RX_RCVD        (byte)0x90 // RX Received (someone sent a payload to us)
#define ZB_FRAME_TYPE_RX_IOD_RCVD    (byte)0x92 // RX IO Data Received
#define ZB_FRAME_TYPE_NODE_ID_IND    (byte)0x95 // Node ID Indicator
#define ZB_FRAME_TYPE_REMCMD_RESP    (byte)0x95 // Remote Command Response


byte ZBframeIDnumTX = 0;  // 0 to 255, this is allowed to roll over to 0 again
byte ZBframeIDnumRX = 0;  // 0 to 255, this is allowed to roll over to 0 again

#define ZB_64ADDR_BCAST {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff} //64bit BCAST is 0x000000000000ffff
#define ZB_64ADDR_BCAST_HIGH (unsigned long)0x00000000 //64bit BCAST is 0x000000000000ffff
#define ZB_64ADDR_BCAST_LOW  (unsigned long)0x0000ffff //64bit BCAST is 0x000000000000ffff
//#define ZB_64ADDR_DEST ZB_64ADDR_BCAST
//#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x3a} // c1at is 0013a20040e6ec3a
//#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x3a} // r2api is 0013a20040e6ec3a
#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x86} // r3api is 0013a20040e6ec86
//#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x92} // r10at is 0013a20040e6ec92
#define ZB_64ADDR_NUM_BYTES 8
byte ZB_64addrDest[ZB_64ADDR_NUM_BYTES]  = ZB_64ADDR_DEST; // destination addr
//byte ZB_64addrDest[ZB_64ADDR_NUM_BYTES]  = ZB_64ADDR_BCAST; // BCAST destination addr
byte ZB_64addrSrc[ZB_64ADDR_NUM_BYTES]  = ZB_64ADDR_BCAST; // source addr

#define ZB_16ADDR_BCAST {0xff, 0xfe} // 16addr bcast or unknown
#define ZB_16ADDR_BCAST_INT (int)0xfffe   // 16addr bcast or unknown
#define ZB_16ADDR_NUM_BYTES 2
byte ZB_16addrDest[ZB_16ADDR_NUM_BYTES] = ZB_16ADDR_BCAST; // destination addr
byte ZB_16addrSrc[ZB_16ADDR_NUM_BYTES]  = ZB_16ADDR_BCAST; // source addr

#define ZB_BCAST_RADIUS (byte)0x00
#define ZB_OPTIONS      (byte)0x00

byte ZB_frameChkSum = 0; //calculate Zigbee Frame checksum here


#define ZB_TX_FRM_DELMTR_BYTES   0x01
#define ZB_TX_FRM_LEN_BYTES      0x02
#define ZB_TX_FRM_TYPE_BYTES     0x01
#define ZB_TX_FRM_ID_BYTES       0x01
#define ZB_TX_FRM_DADDR64_BYTES  0x08
#define ZB_TX_FRM_DADDR16_BYTES  0x02
#define ZB_TX_FRM_BRADIUS_BYTES  0x01
#define ZB_TX_FRM_OPTIONS_BYTES  0x01
//#define ZB_TX_FRM_HEADER_BYTES  17
#define ZB_TX_FRM_HEADER_BYTES  (ZB_TX_FRM_DELMTR_BYTES + ZB_TX_FRM_LEN_BYTES + ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_ID_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_BRADIUS_BYTES + ZB_TX_FRM_OPTIONS_BYTES)

#define ZB_FRM_PAYLOAD_BYTES    12  // fixed number in this design
#define ZB_TX_FRM_PAYLOAD_BYTES ZB_FRM_PAYLOAD_BYTES
#define ZB_TX_FRM_CHKSUM_BYTES  1
#define ZB_TX_FRM_BYTES   (ZB_TX_FRM_HEADER_BYTES + ZB_TX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES)
#define ZB_TX_FRM_BYTES_SH_MAX   (ZB_TX_FRM_HEADER_BYTES + ZB_TX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES) // max bytes SmartHome nodes should expect to receive in a ZB frame

#define ZB_TX_FRM_LEN  (int)(ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_ID_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_BRADIUS_BYTES + ZB_TX_FRM_OPTIONS_BYTES + ZB_FRM_PAYLOAD_BYTES)

// Common frame offsets
#define ZB_FRM_OFFSET_DELMTR     0
#define ZB_FRM_OFFSET_LENH       1
#define ZB_FRM_OFFSET_LENL       2
#define ZB_FRM_OFFSET_FTYPE      3

// TX Request frame offsets
#define ZB_FRM_OFFSET_FID        4
#define ZB_FRM_OFFSET_TX_DADDR64BH  5
#define ZB_FRM_OFFSET_TX_DADDR64B7  5
#define ZB_FRM_OFFSET_TX_DADDR64B6  6
#define ZB_FRM_OFFSET_TX_DADDR64B5  7
#define ZB_FRM_OFFSET_TX_DADDR64B4  8
#define ZB_FRM_OFFSET_TX_DADDR64BL  9
#define ZB_FRM_OFFSET_TX_DADDR64B3  9
#define ZB_FRM_OFFSET_TX_DADDR64B2  10
#define ZB_FRM_OFFSET_TX_DADDR64B1  11
#define ZB_FRM_OFFSET_TX_DADDR64B0  12
#define ZB_FRM_OFFSET_TX_DADDR16H   13
#define ZB_FRM_OFFSET_TX_DADDR16L   14
#define ZB_FRM_OFFSET_TX_BRADIUS    15
#define ZB_FRM_OFFSET_TX_OPTIONS    16
#define ZB_FRM_OFFSET_TX_PAYLOAD    17
#define ZB_FRM_OFFSET_TX_CHKSUM     (ZB_FRM_OFFSET_TX_PAYLOAD + ZB_FRM_PAYLOAD_BYTES)

// RX Received frame does NOT have BRADIUS or FrameID bytes, and so is 2 bytes shorter than a TXreq frame!!!
#define ZB_FRM_OFFSET_RX_SADDR64BH  4
#define ZB_FRM_OFFSET_RX_SADDR64B7  4
#define ZB_FRM_OFFSET_RX_SADDR64B6  5
#define ZB_FRM_OFFSET_RX_SADDR64B5  6
#define ZB_FRM_OFFSET_RX_SADDR64B4  7
#define ZB_FRM_OFFSET_RX_SADDR64BL  8
#define ZB_FRM_OFFSET_RX_SADDR64B3  8
#define ZB_FRM_OFFSET_RX_SADDR64B2  9
#define ZB_FRM_OFFSET_RX_SADDR64B1  10
#define ZB_FRM_OFFSET_RX_SADDR64B0  11
#define ZB_FRM_OFFSET_RX_SADDR16H   12
#define ZB_FRM_OFFSET_RX_SADDR16L   13
#define ZB_FRM_OFFSET_RX_OPTIONS    14
#define ZB_FRM_OFFSET_RX_PAYLOAD    15
#define ZB_FRM_OFFSET_RX_CHKSUM     (ZB_FRM_OFFSET_RX_PAYLOAD + ZB_FRM_PAYLOAD_BYTES)





typedef struct
{
    volatile unsigned int SHdestID;     // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile unsigned int SHsrcID;      // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile byte         SHmsgType;    // 8bit Smarthome message type
    volatile byte         SHcommand;    // 8bit Smarthome command
    volatile byte         SHstatusH;    // High Byte of 16bit Smarthome 
    volatile byte         SHstatusL;    // Low byte of 16bbit Smarthome
    volatile byte         SHstatusVal;    // 8bit Smarthome message type
    volatile byte         SHpayldCRC;   // 8bit Smarthome message type
    volatile byte         SHreserved1;  // 8bit Smarthome message type
    volatile byte         SHreserved2;  // 8bit Smarthome message type
} SHpayload, *prtSHpayload;
unsigned int rxDestID = 0;
unsigned int rxSrcID = 0;
byte         rxMsgType = 0;
byte         rxCommand = 0;
unsigned int rxStatusID = 0; //16bits
byte         rxStatusH = 0; // 8bits High of rxStatusID
byte         rxStatusL = 0; // 8bits Low of rxStatusID
byte         rxStatusVal = 0;   
byte         rxPayldCRC = 0;  
byte         rxReserved1 = 0; 
byte         rxReserved2 = 0; 


typedef struct
{
    volatile byte ZBfrmDelimiter;
    volatile unsigned int  ZBfrmLength;
    volatile byte ZBfrmType;
    volatile byte ZBfrmID;
    volatile unsigned long ZBdaddr64High;
    volatile unsigned long ZBdaddr64Low;
    volatile unsigned int  ZBdaddr16;
    volatile byte ZBfrmRadius;
    volatile byte ZBfrmOptions;
    //volatile byte ZBfrmPayload[ZB_FRM_PAYLOAD_BYTES];
    volatile SHpayload ZBfrmPayload;
    //SHpayload     ZBfrmPayload;
    volatile byte ZBfrmChksum;;
} ZBframeTX, *prtZBframeTX;

ZBframeTX    myZBframeTX;
prtZBframeTX ptrMyZBframeTX = &myZBframeTX;
byte *txBuffer = (byte *)&myZBframeTX; //get a pointer to byte which points to our Zigbee Frame stuct, to treat it as TX buffer for Serial.write

typedef struct
{
    volatile byte ZBfrmDelimiter;
    volatile unsigned int  ZBfrmLength;
    volatile byte ZBfrmType;
    volatile unsigned long ZBsaddr64High;
    volatile unsigned long ZBsaddr64Low;
    volatile unsigned int  ZBsaddr16;
    volatile byte ZBfrmOptions;
    //volatile byte ZBfrmPayload[ZB_FRM_PAYLOAD_BYTES];
    SHpayload     ZBfrmPayload;
    volatile byte ZBfrmChksum;;
} ZBframeRX, *prtZBframeRX;

ZBframeRX    myZBframeRX; //global variable to use
prtZBframeRX ptrMyZBframeRX = &myZBframeRX; //global pointer to use
byte *rxBuffer = (byte *)&myZBframeRX; //get a pointer to byte (ie. byte array) which points to our Zigbee Frame stuct, to treat it as RX buffer for Serial.read

#define ZB_IN_FRAME_YES 1
#define ZB_IN_FRAME_NO  0

byte ZBinFrameTX = ZB_IN_FRAME_NO;
byte ZBoffsetTXbuff = 0; // byte counter, delimiter byte is number 0. Used as offset into txBuffer while creating a frame to transmit

byte ZBinFrameRX = ZB_IN_FRAME_NO;
byte ZBoffsetRXbuff = 0; // byte counter, delimiter byte is number 0. Used as offset into rxBuffer while receiving a frame

byte newFrameRXed = 0;   // have received a new RX Received frame, waiting for processing
byte newFrameRXstatus = 0;  // 1 if success (got full frame and ZB chksum matches our calc)
byte newFrameForTX = 0;  // have a new TX Request frame ready to send out on uart/serial port


byte i=0; //for loop counter

// standard pin13 LED on Arduino Uno and Due for testing and debug. NOT compatible with LCS panel installed on Due nodes.
int ledPin = 13;                 // LED connected to digital pin 13 for debug
byte ledPinState = 0;



void setup() {
    // put your setup code here, to run once:
    pinMode(ledPin, OUTPUT);      // sets the digital pin as output
    ledPinState = 0;

    //initialize Zigbee frame tracking
    ZBinFrameRX = ZB_IN_FRAME_NO;
    newFrameRXed = NO;   
    newFrameForTX = NO;  
    
    // Xbee should be preconfigured for API mode, 9600, 8n1, so match that in Arduino serial port
    Serial.begin(9600);
  
    // say hi to xbee r10at
    // 7E 00 10 10 01 00 13 A2 00 40 E6 EC 92 FF FE 00 00 68 69 C7

    // say hi to xbee r5at
    // 7E 00 10 10 01 00 13 A2 00 40 E3 50 42 FF FE 00 00 68 69 B6
  
    // say hi to xbee c1at
    // 7E 00 10 10 01 00 13 A2 00 40 E6 EC 3A FF FE 00 00 68 69 1F

    // say hi to all - broadcast
    // 7E 00 10 10 01 00 00 00 00 00 00 FF FF FF FE 00 00 68 69 22
  
    //zbXmitAPIframe();
    //zbExperimentalXmitAPIframe();
//    zbExperimentalXmitAPIframe2();
}

void loop() {
  // put your main code here, to run repeatedly:

  // Check if have pushbutton input

  // Check if have a Zigbee message - interrupt from uart

  // Check if have a touchscreen input - interrupt from ???

//  digitalWrite(ledPin, HIGH);   // sets the LED on
//  delay(1000);                  // waits for a second
//  digitalWrite(ledPin, LOW);    // sets the LED off
//  delay(1000);                  // waits for a second
  
    //experimenting with receive
    //zbRcvFrame();

    if(newFrameRXed == YES)
    {
        processRXframe();
    }
    else // NO
    {
        //check for a new incoming frame
        zbRcvFrame();
    }

    if(newFrameForTX == YES)
    {
        
    }

}


/*
 * Xbee class for Zigbee messaging
 * 
 * Zigbee API mode message is
 * 0x7e
 */
//class Xbee
//{
//  private:
//    byte 
//  public:
//    byte xbSndCommand();
//    byte xbSndAck();
//    byte xbSndConfirm();
//    byte xbSndComplete();
//    byte xbRcvCommand();
//    byte xbRcvAck();
//    byte xbRcvConfirm();
//    byte xbRcvComplete();
//}


// Have a conversation with another ZB node.
//
// Each conversation is 4 Zigbee messages:
//    1. Source node sends command to recipient
//    2. Recipient node Acknowledges command received, asks Source node for confirmation
//    3. Source node sends confirmation
//    4. Recipient node indicates completion success or failure
// Istarted = 1 if this node begins the conversation
//          = 0 if another node began the conversation
 
void doConversation(void)
{
  
}


// Transmit a Zigbee API Frame
// Each Zigbee message frame has a 12byte payload, so length=23=0x17 bytes, total frame=27bytes
void zbXmitAPIframe(void)
{
    //init checksum for new Zigbee Frame
    ZB_frameChkSum = 0;
    
    Serial.write(ZB_START_DELIM); // Start delimiter
    //  Serial.flush();
    Serial.write(0x00); // Hbyte API frame length 
    Serial.write(0x17); // Lbyte API frame length

    // Start computing Zigbee Frame checksum here:
    Serial.write(ZB_FRAME_TYPE_TX_REQ); // Frame Type Transmit Request 0x10
    ZB_frameChkSum += ZB_FRAME_TYPE_TX_REQ;
    Serial.write(ZBframeIDnumTX); // Frame ID number
    ZB_frameChkSum += ZBframeIDnumTX;
    //  Serial.flush();

    // loop to send all bytes that make up the 64bit Zigbee destination address
    for(i=0; i<ZB_64ADDR_NUM_BYTES; i++)
    {
        Serial.write(ZB_64addrDest[i]);
        ZB_frameChkSum += ZB_64addrDest[i];
    }

    // loop to send all bytes that make up the 16bit Zigbee destination address
    for(i=0; i<ZB_16ADDR_NUM_BYTES; i++)
    {
        Serial.write(ZB_16addrDest[i]);
        ZB_frameChkSum += ZB_16addrDest[i];
    }

    Serial.write(ZB_BCAST_RADIUS); // Zigbee broadcast Radius
    ZB_frameChkSum += ZB_BCAST_RADIUS;
    Serial.write(ZB_OPTIONS); // Zigbee Frame Options
    ZB_frameChkSum += ZB_OPTIONS;

    // Message payload, book calls this "RF data"
        // 2byte target load ID
        // loop to send all bytes that make up the 16bit Zigbee destination address
        for(i=0; i<SH_NODE_ID_NUM_BYTES; i++)
        {
            Serial.write(SH_DestID[i]);
            ZB_frameChkSum += SH_DestID[i];
        }

        // 2bytes sender node ID
        for(i=0; i<SH_NODE_ID_NUM_BYTES; i++)
        {
            Serial.write(SH_SourceID[i]);
            ZB_frameChkSum += SH_SourceID[i];
        }

        // 1byte command
        Serial.write(SH_command); // SmartHome node command
        ZB_frameChkSum += SH_command;

        // 2bytes node ID for status
        for(i=0; i<SH_NODE_ID_NUM_BYTES; i++)
        {
            Serial.write(SH_StatusID[i]);
            ZB_frameChkSum += SH_StatusID[i];
        }
        // 1byte status value
        Serial.write(SH_nodeStatus); // SmartHome node command
        ZB_frameChkSum += SH_nodeStatus;

        // 1byte crc
        Serial.write(SH_msgCRC); // SmartHome message CRC
        ZB_frameChkSum += SH_msgCRC;

        // 2bytes reserved for future use, use 0x00, 0x00 for now
        Serial.write(SH_RESERVED_BYTE); // SmartHome message RESERVED BYTE
        ZB_frameChkSum += SH_RESERVED_BYTE;
        Serial.write(SH_RESERVED_BYTE); // SmartHome message RESERVED BYTE
        ZB_frameChkSum += SH_RESERVED_BYTE;

    // STOP computing Zigbee API Frame checksum here.

    Serial.write(ZB_frameChkSum); // Zigbee API Frame Checksum value

}

void zbExperimentalXmitAPIframe(void)
{
    //init checksum for new Zigbee Frame
    ZB_frameChkSum = 0;

    ZBframeIDnumTX = 1;
    //ZB_64addrDest = ZB_64ADDR_BCAST;
    //(byte)ZB_64addrDest[ZB_64ADDR_NUM_BYTES] = ZB_64ADDR_BCAST;
    
    Serial.write(ZB_START_DELIM); // Start delimiter
    //  Serial.flush();
    Serial.write(0x00); // Hbyte API frame length 
    Serial.write(0x12); // Lbyte API frame length

    // Start computing Zigbee Frame checksum here:
    Serial.write(ZB_FRAME_TYPE_TX_REQ); // Frame Type Transmit Request 0x10
    ZB_frameChkSum += ZB_FRAME_TYPE_TX_REQ;
    Serial.write(ZBframeIDnumTX); // Frame ID number
    ZB_frameChkSum += ZBframeIDnumTX;
    //  Serial.flush();

    // loop to send all bytes that make up the 64bit Zigbee destination address
    for(i=0; i<ZB_64ADDR_NUM_BYTES; i++)
    {
        Serial.write(ZB_64addrDest[i]);
        ZB_frameChkSum += ZB_64addrDest[i];
    }

    // loop to send all bytes that make up the 16bit Zigbee destination address
    for(i=0; i<ZB_16ADDR_NUM_BYTES; i++)
    {
        Serial.write(ZB_16addrDest[i]);
        ZB_frameChkSum += ZB_16addrDest[i];
    }

    Serial.write(ZB_BCAST_RADIUS); // Zigbee broadcast Radius
    ZB_frameChkSum += ZB_BCAST_RADIUS;
    Serial.write(ZB_OPTIONS); // Zigbee Frame Options
    ZB_frameChkSum += ZB_OPTIONS;

    // Message payload, book calls this "RF data"
        Serial.write('t'); // SmartHome message RESERVED BYTE
        ZB_frameChkSum += 't';
        Serial.write('e'); // SmartHome message RESERVED BYTE
        ZB_frameChkSum += 'e';
        Serial.write('s'); // SmartHome message RESERVED BYTE
        ZB_frameChkSum += 's';
        Serial.write('t'); // SmartHome message RESERVED BYTE
        ZB_frameChkSum += 't';

    // STOP computing Zigbee API Frame checksum here.

    //Serial.write(ZB_frameChkSum); // Zigbee API Frame Checksum value
    Serial.write((byte)33); // Zigbee API Frame Checksum value

}


void zbExperimentalXmitAPIframe2(void)
{
//    byte *txBuffer = (byte *)&myZBframeTX; //get a pointer to byte which points to our Zigbee Frame stuct, to treat it as TX buffer for Serial.write
    //byte *txBuffer;
    //txBuffer = (byte *)&myZBframeTX;
    byte testPayload[12] = { 'H', 'i', '_', 'f', 'r', 'o', 'm', '_', 'U', 'n', 'o', '!' };
    
    //init checksum for new Zigbee Frame
    myZBframeTX.ZBfrmChksum = 0;

    //myZBframeTX.ZBfrmID = 1;

    myZBframeTX.ZBfrmDelimiter = ZB_START_DELIM;         // Zigbee Start delimiter
    myZBframeTX.ZBfrmLength   = ZB_TX_FRM_LEN;          // Zigbee API frame length is num bytes BETWEEN this length and checksum
    myZBframeTX.ZBfrmType     = ZB_FRAME_TYPE_TX_REQ;    // Transmit Request
    myZBframeTX.ZBfrmID       = (byte)0x31; //ascii '1' //1;
    myZBframeTX.ZBdaddr64High = ZB_64ADDR_BCAST_HIGH;   // 64bit dest addr Hich part
    myZBframeTX.ZBdaddr64Low  = ZB_64ADDR_BCAST_LOW;    // 64bit dest addr Low part
    myZBframeTX.ZBdaddr16     = ZB_16ADDR_BCAST_INT;    // 16bit dest addr
    myZBframeTX.ZBfrmRadius   = ZB_BCAST_RADIUS;
    myZBframeTX.ZBfrmOptions  = ZB_OPTIONS;

    for(i=0; i<ZB_FRM_PAYLOAD_BYTES; i++)
    {
//        myZBframeTX.ZBfrmPayload[i] = testPayload[i];
    }
    
    // calculate frame checksum
    myZBframeTX.ZBfrmChksum = (byte)25;

    //Serial.write(myZBframeTX, (byte)ZB_TX_FRM_BYTES) ;
    //Serial.write(myZBframeTX, sizeof(ZBframe)) ;
    Serial.write(txBuffer, sizeof(ZBframeTX)) ;
    //Serial.flush();
}



int zbRcvFrame(void)
{
    byte ZB_frm_byte = 0; // byte received in uart RX by Serial.read    
    byte ZBfrmRXchmsumCalc = 0;
    byte ZBchksumFromSender= 0;
        
    while(Serial.available() > 0)
    {
        // Read a byte from uart RX buffer
        ZB_frm_byte = Serial.read(); 
        //flipLEDpin(); // change LED when get a byte   
            
        if((ZBinFrameRX == ZB_IN_FRAME_NO) && (ZB_frm_byte == ZB_START_DELIM)) 
        {
            // beginning a new frame
            ZBinFrameRX = ZB_IN_FRAME_YES;
            ZBoffsetRXbuff = 0; //Delimiter byte is offset 0 into RX buffer
//            Serial.print(ZB_frm_byte, HEX); // debug print integer to serial port monitor            
        }
        else if(ZBinFrameRX == ZB_IN_FRAME_NO)  // and new byte is NOT ZB_START_DELIM
        {
            // NOT already in a frame, and NOT starting a new frame here, ignore unknown bytes in RX
            return(0);
        }
        //else //in a frame
        else if (ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_CHKSUM)
        {
            //put received byte into rx buffer (aka received Zigbee frame structure)
            rxBuffer[ZBoffsetRXbuff] = ZB_frm_byte;
            ZBchksumFromSender = ZB_frm_byte;

            //final checksum is ff - the sum of bytes 3 to N-1 (excludes the received checksum value)
            ZBfrmRXchmsumCalc = 0xff - ZBfrmRXchmsumCalc;

            // pull out our SmartHome data items into global vars
            extractRXpayload();

            // let rest of program know that a new RX frame is waiting to be processed
            newFrameRXed = YES;
            
            // end of frame, next byte received is NOT part of this same frame
            ZBinFrameRX = ZB_IN_FRAME_NO;
            return(0);
        }


        if(ZBinFrameRX == ZB_IN_FRAME_YES)
        {
            //for debug
            //Serial.print(ZB_frm_byte, HEX); // debug print integer to serial port monitor            

            //put received byte into rx buffer (aka received Zigbee frame structure)
            rxBuffer[ZBoffsetRXbuff] = ZB_frm_byte;
            
            if((ZBoffsetRXbuff >= ZB_FRM_OFFSET_FTYPE) && (ZBoffsetRXbuff < ZB_FRM_OFFSET_RX_CHKSUM))
            {
                // add to the checksum for this frame
                ZBfrmRXchmsumCalc += ZB_frm_byte;             
            }

            //increment offset for next byte received in this frame (AFTER checking if should add to checksum)
            ZBoffsetRXbuff++ ;
        }
    }    
}


// pull the SmartHome message items out of Zigbee frame payload and save to global variables
void extractRXpayload(void)
{
    rxDestID = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHdestID);
    rxSrcID = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHsrcID);
    rxMsgType = myZBframeRX.ZBfrmPayload.SHmsgType;
    rxCommand = myZBframeRX.ZBfrmPayload.SHcommand;
    rxStatusH = myZBframeRX.ZBfrmPayload.SHstatusH;
    rxStatusL = myZBframeRX.ZBfrmPayload.SHstatusL;
    //rxStatusID = BYTESWAP16( (unsigned int)myZBframeRX.ZBfrmPayload.SHstatusH ); //uint16 gets both SHstatusH and SHstatusL
    rxStatusID = (rxStatusH << 8) | rxStatusL; //uint16 gets both SHstatusH and SHstatusL
    rxStatusVal= myZBframeRX.ZBfrmPayload.SHstatusVal;
    rxPayldCRC = myZBframeRX.ZBfrmPayload.SHpayldCRC;
    rxReserved1 = myZBframeRX.ZBfrmPayload.SHreserved1;
    rxReserved2 = myZBframeRX.ZBfrmPayload.SHreserved2;  
}


// parse the received ZB frame payload to determine if message was for this node or not
// and if it was, determine what to do as result
void processRXframe(void)
{
    //debugPrintRxBuffer();
    //flipLEDpin();


    //check if RXdestID is this node cotnroller or one of its target loads
    //go through dest IDs stored in SD/eeprom (NV) and compare with RXdestID in message


    // if RXdestID is in this node, figure out if node control or which target

        // run the received SH command
        SHrunCommand();
    
    // We processed our new RX frame, so no longer have a new one
    // Last thing to do when processing an RX frame, so we're ready to receive another new frame
    newFrameRXed = NO;
}


void SHrunCommand(void)
{
        switch ( rxCommand ) 
        {
            case SH_CMD_LOAD_ON:
                // Code
                digitalWrite(ledPin, HIGH);   // sets the LED on
                break;

            case SH_CMD_LOAD_OFF:
                // Code
                digitalWrite(ledPin, LOW);   // sets the LED off
               break;

            case SH_CMD_NOP:  // No OPeration, DO NOTHING (same as default)
            default:
                // Unknown command, do nothing
               break;
        }

        Serial.println("");
        Serial.print("Ran SH cmd code 0x");
        Serial.print(rxCommand, HEX);
        Serial.println("");

        // Send notice message to Server for logging what SmartHome command we received to run
        
}

void debugPrintRxBuffer(void)
{
    Serial.println("");
    Serial.print("Received a Zigbee buffer containing payload data -> ");
    for(i=0; i<=ZB_FRM_OFFSET_RX_CHKSUM; i++)
    {
        Serial.print(rxBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
}



// for debug
// change value to LED on pin 13, to turn it on or turn it off
void flipLEDpin(void)
{
    if(ledPinState == 0)
    {
        ledPinState = 1;
        digitalWrite(ledPin, HIGH);   // sets the LED on
    }
    else
    {
        ledPinState = 0;
        digitalWrite(ledPin, LOW);   // sets the LED off
    }
    
}

