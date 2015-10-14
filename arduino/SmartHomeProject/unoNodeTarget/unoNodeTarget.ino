
// uncomment these DEBUG items to enable debug serial prints
//#define DEBUG_ZB_RX
//#define DEBUG_ZB_TX


#include "Arduino.h"

// Because Unos and Dues have different sizes for int
#include "stdint.h"


// The following LGPL 2.1+ byteswap macros slightly modified from 
// http://repo-genesis3.cbi.utsa.edu/crossref/ns-sli/usr/include/bits/byteswap.h.html
// These byteswaps are needed, as 16bit ints and 32bit longs in the Zigbee message byte order are byteswapped 
// compared to the Arduino's requirements to conveniently use them as 16bit int or 32bit long values, 
// rather than doing more work to deal with everythign as individual bytes
#define BYTESWAP16(x) \
       (uint16_t)((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

#define BYTESWAP32(x) \
       (uint32_t)((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |               \
                       (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))

#define  NO 0x00
#define YES 0x01


// The 4 types of SmartHome message to be sent over Zigbee as payload.SHmsgType
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
uint8_t SH_TargetLoadID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // destination addr
//uint8_t SH_DestID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // destination node ID
//uint8_t SH_SourceID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // source node ID
uint8_t SH_DestID[SH_NODE_ID_NUM_BYTES] = {'d', 'u'}; // destination node ID
uint8_t SH_SourceID[SH_NODE_ID_NUM_BYTES] = {'d', 'e'}; // source node ID
uint8_t SH_StatusID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // node addr for Status
uint8_t SH_command = SH_CMD_NOP;
uint8_t SH_nodeStatus = 0x00;
uint8_t SH_msgCRC = 0x00;

// Temporary Node ID method, until they can be read from microSD, EEPROM etc.
// Eventually, Unos will read from EEPROM and Dues will read from SD.

#define IDnode      0x01
#define IDloadLight 0x01
#define IDloadFan   0x02

#define ZB_ID_COORD 0x00        // The Zigbee coordinator is always ID 0
#define ZB_ID_PAN   0x42        // The ID for the local network. Other networks are on different PAN IDs.
//#define ZB_ID_H     (uint8_t){0x00, 0x13, 0xa2, 0x00}  // The high part of Zigbee ID for Digi Xbee modules is fixed at 0x0013a200
//uint8_t zb_id_L[] = {0x40, 0xe6, 0xec, 0x86};        // 0x40e6ec86

// Zigbee API frame stuff
#define ZB_START_DELIMITER               (uint8_t)0x7e // indicates start of a Zigbee frame

#define ZB_FRAME_TYPE_ATCMD_IMD      (uint8_t)0x08 // Transmit Request
#define ZB_FRAME_TYPE_ATCMD_QUE      (uint8_t)0x09 // Transmit Request
#define ZB_FRAME_TYPE_REMCMD_REQ     (uint8_t)0x17 // Transmit Request
#define ZB_FRAME_TYPE_ATCMD_RESP     (uint8_t)0x88 // Transmit Request
#define ZB_FRAME_TYPE_MDM_ST         (uint8_t)0x8a // Modem Status
#define ZB_FRAME_TYPE_TX_REQ         (uint8_t)0x10 // Transmit Request (send a payload)
#define ZB_FRAME_TYPE_TX_RESP        (uint8_t)0x8b // Transmit Response
#define ZB_FRAME_TYPE_RX_RCVD        (uint8_t)0x90 // RX Received (someone sent a payload to us)
#define ZB_FRAME_TYPE_RX_IOD_RCVD    (uint8_t)0x92 // RX IO Data Received
#define ZB_FRAME_TYPE_NODE_ID_IND    (uint8_t)0x95 // Node ID Indicator
#define ZB_FRAME_TYPE_REMCMD_RESP    (uint8_t)0x95 // Remote Command Response


uint8_t ZBframeIDnumTX = 0;  // 0 to 255, this is allowed to roll over to 0 again
uint8_t ZBframeIDnumRX = 0;  // 0 to 255, this is allowed to roll over to 0 again

#define ZB_64ADDR_BCAST {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff} //64bit BCAST is 0x000000000000ffff
#define ZB_64ADDR_BCAST_HIGH (uint32_t)0x00000000 //64bit BCAST is 0x000000000000ffff
#define ZB_64ADDR_BCAST_LOW  (uint32_t)0x0000ffff //64bit BCAST is 0x000000000000ffff
//#define ZB_64ADDR_DEST ZB_64ADDR_BCAST
//#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x3a} // c1at is 0013a20040e6ec3a
//#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x3a} // r2api is 0013a20040e6ec3a
#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x86} // r3api is 0013a20040e6ec86
//#define ZB_64ADDR_DEST {0x00, 0x13, 0xa2, 0x00, 0x40, 0xe6, 0xec, 0x92} // r10at is 0013a20040e6ec92
#define ZB_64ADDR_NUM_BYTES 8
uint8_t ZB_64addrDest[ZB_64ADDR_NUM_BYTES]  = ZB_64ADDR_DEST; // destination addr
//uint8_t ZB_64addrDest[ZB_64ADDR_NUM_BYTES]  = ZB_64ADDR_BCAST; // BCAST destination addr
uint8_t ZB_64addrSrc[ZB_64ADDR_NUM_BYTES]  = ZB_64ADDR_BCAST; // source addr

#define ZB_16ADDR_BCAST {0xff, 0xfe} // 16addr bcast or unknown
#define ZB_16ADDR_BCAST_INT (uint16_t)0xfffe   // 16addr bcast or unknown
#define ZB_16ADDR_NUM_BYTES 2
uint8_t ZB_16addrDest[ZB_16ADDR_NUM_BYTES] = ZB_16ADDR_BCAST; // destination addr
uint8_t ZB_16addrSrc[ZB_16ADDR_NUM_BYTES]  = ZB_16ADDR_BCAST; // source addr

#define ZB_BCAST_RADIUS (uint8_t)0x00
#define ZB_OPTIONS      (uint8_t)0x00

uint8_t ZB_frameChkSum = 0; //calculate Zigbee Frame checksum here

// TODO clean up TX vs RX when are same thing (LEN bytes, ADDR64 bytes etc)
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
#define ZB_RX_FRM_HEADER_BYTES  (ZB_TX_FRM_DELMTR_BYTES + ZB_TX_FRM_LEN_BYTES + ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_OPTIONS_BYTES)

#define ZB_FRM_PAYLOAD_BYTES    12  // fixed number in this design
#define ZB_TX_FRM_PAYLOAD_BYTES ZB_FRM_PAYLOAD_BYTES
#define ZB_TX_FRM_CHKSUM_BYTES  1
#define ZB_TX_FRM_BYTES   (ZB_TX_FRM_HEADER_BYTES + ZB_TX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES)
#define ZB_TX_FRM_BYTES_INT     (uint16_t)ZB_TX_FRM_BYTES
#define ZB_TX_FRM_BYTES_SH_MAX   (ZB_TX_FRM_HEADER_BYTES + ZB_TX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES) // max bytes SmartHome nodes should expect to receive in a ZB frame

#define ZB_RX_FRM_PAYLOAD_BYTES ZB_FRM_PAYLOAD_BYTES
#define ZB_RX_FRM_BYTES         (uint16_t)(ZB_RX_FRM_HEADER_BYTES + ZB_RX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES)
#define ZB_RX_FRM_BYTES_INT     (uint16_t)ZB_RX_FRM_BYTES

#define ZB_TX_FRM_LEN  (uint16_t)(ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_ID_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_BRADIUS_BYTES + ZB_TX_FRM_OPTIONS_BYTES + ZB_FRM_PAYLOAD_BYTES)
#define ZB_RX_FRM_LEN  (uint16_t)(ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_OPTIONS_BYTES + ZB_FRM_PAYLOAD_BYTES)

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
    volatile uint16_t SHdestID;     // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint16_t SHsrcID;      // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint8_t         SHmsgType;    // 8bit Smarthome message type
    volatile uint8_t         SHcommand;    // 8bit Smarthome command
    volatile uint8_t         SHstatusH;    // High Byte of 16bit Smarthome 
    volatile uint8_t         SHstatusL;    // Low byte of 16bbit Smarthome
    volatile uint8_t         SHstatusVal;    // 8bit Smarthome message type
    volatile uint8_t         SHpayldCRC;   // 8bit Smarthome message type
    volatile uint8_t         SHreserved1;  // 8bit Smarthome message type
    volatile uint8_t         SHreserved2;  // 8bit Smarthome message type
} SHpayload, *prtSHpayload;

//change these global vars to another instance of SHpayload struct for TX and again for TX
uint16_t SHdestIDrx = 0;
uint16_t SHsrcIDrx = 0;
uint8_t         SHmsgTypeRX = 0;
uint8_t         SHcommandRX = 0;
uint16_t SHstatusIDrx = 0; //16bits
uint8_t         SHstatusHrx = 0; // 8bits High of SHstatusIDrx
uint8_t         SHstatusLrx = 0; // 8bits Low of SHstatusIDrx
uint8_t         SHstatusValRX = 0;   
uint8_t         SHchksumRX = 0;  
uint8_t         SHreserved1rx = 0; 
uint8_t         SHreserved2rx = 0; 


typedef struct
{
    volatile uint8_t ZBfrmDelimiter;
    volatile uint16_t  ZBfrmLength;
    volatile uint8_t ZBfrmType;
    volatile uint8_t ZBfrmID;
    volatile uint32_t ZBdaddr64High;
    volatile uint32_t ZBdaddr64Low;
    volatile uint16_t  ZBdaddr16;
    volatile uint8_t ZBfrmRadius;
    volatile uint8_t ZBfrmOptions;
    //volatile uint8_t ZBfrmPayload[ZB_FRM_PAYLOAD_BYTES];
    volatile SHpayload ZBfrmPayload;
    //SHpayload     ZBfrmPayload;
    volatile uint8_t ZBfrmChksum;;
} ZBframeTX, *prtZBframeTX;

ZBframeTX    myZBframeTX;
prtZBframeTX ptrMyZBframeTX = &myZBframeTX;
uint8_t *txBuffer = (uint8_t *)&myZBframeTX; //get a pointer to byte which points to our Zigbee Frame stuct, to treat it as TX buffer for Serial.write

typedef struct
{
    volatile uint8_t ZBfrmDelimiter;
    volatile uint16_t  ZBfrmLength;
    volatile uint8_t ZBfrmType;
    volatile uint32_t ZBsaddr64High;
    volatile uint32_t ZBsaddr64Low;
    volatile uint16_t  ZBsaddr16;
    volatile uint8_t ZBfrmOptions;
    //volatile uint8_t ZBfrmPayload[ZB_FRM_PAYLOAD_BYTES];
    SHpayload     ZBfrmPayload;
    volatile uint8_t ZBfrmChksum;;
} ZBframeRX, *prtZBframeRX;

ZBframeRX    myZBframeRX; //global variable to use
prtZBframeRX ptrMyZBframeRX = &myZBframeRX; //global pointer to use
uint8_t *rxBuffer = (uint8_t *)&myZBframeRX; //get a pointer to byte (ie. byte array) which points to our Zigbee Frame stuct, to treat it as RX buffer for Serial.read

#define ZB_IN_FRAME_YES 1
#define ZB_IN_FRAME_NO  0

uint8_t ZBinFrameTX = ZB_IN_FRAME_NO;
uint8_t ZBoffsetTXbuff = 0; // byte counter, delimiter byte is number 0. Used as offset into txBuffer while creating a frame to transmit

uint8_t ZBinFrameRX = ZB_IN_FRAME_NO;
uint8_t ZBoffsetRXbuff = 0; // byte counter, delimiter byte is number 0. Used as offset into rxBuffer while receiving a frame

uint8_t newFrameRXed = NO;   // have received a new RX Received frame, waiting for processing
uint8_t newFrameRXstatus = 0;  // 1 if success (got full frame and ZB chksum matches our calc)
uint8_t newFrameForTX = 0;  // have a new TX Request frame ready to send out on uart/serial port

uint8_t newSHmsgRX = NO;

uint16_t ZBfrmLengthRX = 0;
uint8_t ZBfrmRXchksumCalc = 0;

uint8_t userInputEvent = 0; // status for user input events (touchscreen/buttons etc. 1bit per possible event)


uint8_t i=0; //for loop counter

// standard pin13 LED on Arduino Uno and Due for testing and debug. NOT compatible with LCS panel installed on Due nodes.
uint16_t ledPin = 13;                 // LED connected to digital pin 13 for debug
uint8_t ledPinState = 0;



void setup() {
    // put your setup code here, to run once:
    pinMode(ledPin, OUTPUT);      // sets the digital pin as output
    ledPinState = 0;

    //initialize Zigbee frame tracking
    ZBinFrameRX = ZB_IN_FRAME_NO;
    newFrameRXed = NO;   
    newFrameForTX = NO;  

    newSHmsgRX = NO;
    ZBfrmLengthRX = 0;
    ZBfrmRXchksumCalc = 0;
    
    // Xbee should be preconfigured for API mode, 9600, 8n1, so match that in Arduino serial port
    Serial.begin(9600);
  

    zbXmitAPIframe();
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
    //zbRcvAPIframe();

    if(newSHmsgRX == YES)
    {
        processSHmsg();
    }
    else // NO new SH message
    {
        //check for a new incoming frame data
        zbRcvAPIframe();
    }

    if(newFrameForTX == YES)
    {
        // transmit the new TX frame
        zbXmitAPIframe();
    }

    if(userInputEvent != 0)
    {
        // Have some user input event from LCD touchscreen or buttons
        //determine if need to send a command
        //determine if need to update LCD display
    }
    

}


// Transmit a Zigbee API TX Request Frame
// Each Zigbee message frame has a 12byte payload, so length=23=0x17 bytes, total frame=27bytes
void zbXmitAPIframe(void)
{
    myZBframeTX.ZBfrmDelimiter = ZB_START_DELIMITER;
    myZBframeTX.ZBfrmLength    = BYTESWAP16(ZB_TX_FRM_LEN);
    myZBframeTX.ZBfrmType      = ZB_FRAME_TYPE_TX_REQ;
    myZBframeTX.ZBfrmID        = 0x01;
    myZBframeTX.ZBdaddr64High  = BYTESWAP32(ZB_64ADDR_BCAST_HIGH);
    myZBframeTX.ZBdaddr64Low   = BYTESWAP32(ZB_64ADDR_BCAST_LOW);
    myZBframeTX.ZBdaddr16      = BYTESWAP16(ZB_16ADDR_BCAST_INT);
    myZBframeTX.ZBfrmRadius    = ZB_BCAST_RADIUS;
    myZBframeTX.ZBfrmOptions   = ZB_OPTIONS;

    // populate the TX frame payload data (aka RF Data)
    populateTXpayload();

    // initialize the Zigbee API frame checksum before calculating it
    myZBframeTX.ZBfrmChksum = 0x00;    
    zbFrmCalcTxChksum();

    debugPrintTxBuffer();

    for(ZBoffsetTXbuff=0; ZBoffsetTXbuff<ZB_TX_FRM_BYTES; ZBoffsetTXbuff++)
    {
        // TODO change to if > 0 so match the receive bytes style
        while( Serial.availableForWrite() == 0 )
        {
            //wait for room in serial buffer, do nothing while waiting
        }
        Serial.write(txBuffer[ZBoffsetTXbuff]); //Uno
    }
}

void populateTXpayload(void)
{
    // DEBUG data for testing the transmit
    myZBframeTX.ZBfrmPayload.SHdestID = BYTESWAP16(0x0102);
    myZBframeTX.ZBfrmPayload.SHsrcID = BYTESWAP16(0x0304);
    myZBframeTX.ZBfrmPayload.SHmsgType = SH_MSG_TYPE_CMD_REQ;
    myZBframeTX.ZBfrmPayload.SHcommand = SH_CMD_LOAD_ON;
    myZBframeTX.ZBfrmPayload.SHstatusH = 0x07;
    myZBframeTX.ZBfrmPayload.SHstatusL = 0x08;
    myZBframeTX.ZBfrmPayload.SHstatusVal = 0x09;
    myZBframeTX.ZBfrmPayload.SHpayldCRC = 0x0a;
    myZBframeTX.ZBfrmPayload.SHreserved1 = 'T';
    myZBframeTX.ZBfrmPayload.SHreserved2 = 'X';
}

// calculate the Zigbee frame checksum for this API TX Request frame
uint8_t zbFrmCalcTxChksum(void)
{
    uint8_t ZBfrmTXchksumCalc = 0;

    for(ZBoffsetTXbuff=ZB_FRM_OFFSET_FTYPE; ZBoffsetTXbuff<ZB_FRM_OFFSET_TX_CHKSUM; ZBoffsetTXbuff++)
    {
        // add to the checksum for this frame
        ZBfrmTXchksumCalc += txBuffer[ZBoffsetTXbuff];             
    }

    // Final part of Zigbee frame checksum calculation is to subtract current total from 0xff
    // Save into txbuffer for sending
    txBuffer[ZBoffsetTXbuff] = 0xff - ZBfrmTXchksumCalc;

    Serial.print(" <<zbTxChksum=");
    Serial.print(txBuffer[ZB_FRM_OFFSET_TX_CHKSUM], HEX);
    Serial.print(">> ");

    return(txBuffer[ZB_FRM_OFFSET_TX_CHKSUM]);
}


void debugPrintTxBuffer(void)
{
    Serial.println("");
    Serial.print("Ready to send a Zigbee API TX Request frame buffer -> ");
    for(i=0; i<=ZB_FRM_OFFSET_TX_CHKSUM; i++)
    {
        Serial.print(txBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
}


// received frame length value should match ZB_RX_FRM_BYTES
uint8_t zbRcvAPIframe(void)
{
    uint8_t ZB_frm_byte = 0; // byte received in uart RX by Serial.read    
    
    uint8_t ZBchksumFromSender= 0;
        
    while(Serial.available() > 0)
    {
        // Read a byte from uart RX buffer
        ZB_frm_byte = Serial.read(); 
            
        if((ZBinFrameRX == ZB_IN_FRAME_NO) && (ZB_frm_byte == ZB_START_DELIMITER)) 
        {
            // beginning a new frame
            ZBinFrameRX = ZB_IN_FRAME_YES;
            ZBoffsetRXbuff = 0; //Delimiter byte is offset 0 into RX buffer
            ZBfrmRXchksumCalc = 0; //new frame starts new checksum
        }
        else if(ZBinFrameRX == ZB_IN_FRAME_NO)  // and new byte, which is NOT ZB_START_DELIMITER
        {
            // NOT already in a frame, and NOT starting a new frame here, ignore unknown bytes in RX
            return(0);
        }
        //else // already in a frame
        else if (ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_CHKSUM)
        {
            //put received byte into rx buffer (aka received Zigbee frame structure)
            rxBuffer[ZBoffsetRXbuff] = ZB_frm_byte;
            ZBchksumFromSender = ZB_frm_byte;

            //final checksum is ff - the sum of bytes 3 to N-1 (excludes the received checksum value)
            ZBfrmRXchksumCalc = 0xff - ZBfrmRXchksumCalc;

            #if 0
            Serial.print(" rx<");
            Serial.print(ZB_frm_byte, HEX);
            Serial.print("> cs<");
            Serial.print(ZBfrmRXchksumCalc, HEX);
            Serial.print("> ");
            #endif
            
            // pull out our SmartHome data items into global vars
//            extractRXpayload();

            #if 0
// goober
            Serial.println("");
            Serial.print(" <ZBfrmLengthRX=");
            Serial.print(ZBfrmLengthRX, HEX);
            Serial.print(" ?= ZB_RX_FRM_LEN=");
            Serial.print(ZB_RX_FRM_LEN, HEX);
            Serial.print(" || ZBchksumFromSender=");
            Serial.print(ZBchksumFromSender, HEX);
            Serial.print(" ?= ZBfrmRXchksumCalc=");
            Serial.print(ZBfrmRXchksumCalc, HEX);
            Serial.print(" -||- myZBframeRX.ZBfrmLength=");
            Serial.print(myZBframeRX.ZBfrmLength, HEX);
            Serial.println("> ");
            #endif
            
            // check frame length and checksum are correct for this ZB frame
            if( (ZBfrmLengthRX == ZB_RX_FRM_LEN) && 
                (ZBchksumFromSender == ZBfrmRXchksumCalc) )
            {
                Serial.print(" <Ding1> ");
                // Have a valid Zigbee frame of our expected length, assume it is valid SH message
                
                // pull out our SmartHome data items into global vars
                extractRXpayload();
                
                newSHmsgRX = YES;   
            }

            // let rest of program know that a new RX frame is waiting to be processed
            newFrameRXed = YES;
            
            // end of frame, next byte received is NOT part of this same frame
            ZBinFrameRX = ZB_IN_FRAME_NO;
            return(0);
        }


        if(ZBinFrameRX == ZB_IN_FRAME_YES)
        {
            //put received byte into rx buffer (aka received Zigbee frame structure)
            rxBuffer[ZBoffsetRXbuff] = ZB_frm_byte;

            if (ZBoffsetRXbuff == ZB_FRM_OFFSET_LENL)
            {
                // have all bytes of the Zigbee Frame Length field, put this into a uint16_t variable
                ZBfrmLengthRX = BYTESWAP16(myZBframeRX.ZBfrmLength);
            }
            else if((ZBoffsetRXbuff >= ZB_FRM_OFFSET_FTYPE) && (ZBoffsetRXbuff < ZB_FRM_OFFSET_RX_CHKSUM))
            {
                // add to the checksum for this frame
                ZBfrmRXchksumCalc += ZB_frm_byte;             
            }

            #if 0
            Serial.print(" rx<");
            Serial.print(ZB_frm_byte, HEX);
            Serial.print("> cs<");
            Serial.print(ZBfrmRXchksumCalc, HEX);
            Serial.print("> ");
            #endif

            //increment offset for next byte received in this frame (AFTER checking if should add to checksum)
            ZBoffsetRXbuff++ ;
        }
    }    
}


// pull the SmartHome message items out of Zigbee frame payload and save to global variables
void extractRXpayload(void)
{
    SHdestIDrx = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHdestID);
    SHsrcIDrx = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHsrcID);
    SHmsgTypeRX = myZBframeRX.ZBfrmPayload.SHmsgType;
    SHcommandRX = myZBframeRX.ZBfrmPayload.SHcommand;
    SHstatusHrx = myZBframeRX.ZBfrmPayload.SHstatusH;
    SHstatusLrx = myZBframeRX.ZBfrmPayload.SHstatusL;
    //SHstatusIDrx = BYTESWAP16( (uint16_t)myZBframeRX.ZBfrmPayload.SHstatusH ); //uint16 gets both SHstatusH and SHstatusL
    SHstatusIDrx = (SHstatusHrx << 8) | SHstatusLrx; //uint16 gets both SHstatusH and SHstatusL
    SHstatusValRX= myZBframeRX.ZBfrmPayload.SHstatusVal;
    SHchksumRX = myZBframeRX.ZBfrmPayload.SHpayldCRC;
    SHreserved1rx = myZBframeRX.ZBfrmPayload.SHreserved1;
    SHreserved2rx = myZBframeRX.ZBfrmPayload.SHreserved2;  
}


// parse the received ZB frame payload to determine if message was for this node or not
// and if it was, determine what to do as result
void processSHmsg(void)
{
    //debugPrintRxBuffer();

    //check if SHdestIDrx is this node cotnroller or one of its target loads
    //go through dest IDs stored in SD/eeprom (NV) and compare with SHdestIDrx in message


    // if SHdestIDrx is in this node, figure out if node control or which target

        // run the received SH command
        SHrunCommand();
    
    // We processed our new RX frame, so no longer have a new one
    // Last thing to do when processing an RX frame, so we're ready to receive another new frame
    newFrameRXed = NO; // TODO - does this belong here??
    newSHmsgRX = NO;
}


void SHrunCommand(void)
{
        switch ( SHcommandRX ) 
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
        Serial.print(SHcommandRX, HEX);
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

