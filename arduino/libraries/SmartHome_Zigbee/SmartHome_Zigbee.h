#ifndef SH_ZIGBEE_H
#define SH_ZIGBEE_H

// cross-platform consistent integer types
#include "stdint.h"


// For DEBUG only, Load and Wall Control IDs (SmartHome IDs)
#define SH_LOAD_ID_ROOM0_LOAD0 (uint16_t)0x0709
#define SH_LOAD_ID_ROOM0_LOAD1 (uint16_t)0x0a0c
#define SH_WC_ID_ROOM0 (uint16_t)0x0123


#define  NO (uint8_t)0x00
#define YES (uint8_t)0x01



typedef struct
{
    volatile uint16_t SHothrID;
    volatile uint8_t  SHmsgType;
    volatile uint8_t  SHcommand;
    volatile uint8_t  SHstatusH;
    volatile uint8_t  SHstatusL;
    volatile uint16_t SHstatusID;    // 16bit alternative to SHstatusH and SHstatusL but represents same bytes in message
    volatile uint8_t  SHstatusVal;
    volatile uint8_t  SHreserved1;
    volatile uint8_t  SHreserved2;
    volatile uint8_t  SHchksum;      // from this node or defined by another node
    volatile uint8_t  SHcalcChksum;  // checksum calculated for comparison to sender's value
    volatile uint8_t  SHstatusTX;    // status of attempt to transmit a Zigbee API frame
    volatile uint8_t  SHstatusRX;    // status of receiving a Zigbee API frame
} SHmessage, *prtSHmessage;



// SH message protocol stages / states for use in state machine current/next states
#define SH_MSG_ST_IDLE      (uint8_t)0x00  // waiting to receive a command message
#define SH_MSG_ST_CMD_INIT  (uint8_t)0x01  // begin an SH protocol message conversation
#define SH_MSG_ST_ACK_REQ   (uint8_t)0x02  // achnowledge receipt of a CMD_INIT message, request confirmation
#define SH_MSG_ST_CNFRM     (uint8_t)0x03  // confirm that this node sent a CMD_INIT message to indicated destination
#define SH_MSG_ST_COMPLETE  (uint8_t)0x04  // complete the SH protocol conversation, include status value        

// use node-specific versions of these instead, in the nodeInfo msg struct
//uint8_t currentMsgState = SH_MSG_ST_IDLE;
//uint8_t nextMsgState    = SH_MSG_ST_IDLE;

// message type fromt eh SmartHome Message payload content provided by message sender
// The 4 types of SmartHome message to be sent over Zigbee as payload.SHmsgType
// together these four messages ame up one SmartHome protocol "conversation"
#define SH_MSG_TYPE_IDLE      (uint8_t)0x00 // NO active message for this node
#define SH_MSG_TYPE_CMD_REQ   (uint8_t)0x01 // Request recipient to run a new command
#define SH_MSG_TYPE_ACK_CREQ  (uint8_t)0x02 // ACKnowledge new command received, Request Confirmation from Sender
#define SH_MSG_TYPE_CONFIRM   (uint8_t)0x03 // Sender Confirms it did indeed initiate this new command request
#define SH_MSG_TYPE_COMPLETED (uint8_t)0x04 // Recipient indicates command has been executed and completed

// SmartHome node Status Values
#define SH_STATUS_SUCCESS       (uint8_t)0x00
#define SH_STATUS_FAILED        (uint8_t)0x01
#define SH_STATUS_CONFIRMED     (uint8_t)0x02
#define SH_STATUS_NOT_CONFIRMED (uint8_t)0x03


// SH Message protocol fixed status values
#define SH_MSG_STATUS_SUCCESS            0x01  // command completed successfully
#define SH_MSG_STATUS_ERR_INVLD_SENDER   0x02  // addressed sender of command denied sending it
#define SH_MSG_STATUS_ERR_WRNG_RCPT_TYPE 0x03  // recipient of the command is wrong type for command
// such as if wall control received an on/off/intensity command


// SmartHome node Commands
#define SH_CMD_NOP               (uint8_t)0x00 // No OPeration, do nothing
#define SH_CMD_LOAD_ON           (uint8_t)0x01 // Turn on the target load at this node
#define SH_CMD_LOAD_OFF          (uint8_t)0x02 // Turn off the target load at this node
#define SH_CMD_LOAD_INC          (uint8_t)0x03 // increase intensity at target load (brighter/faster)
#define SH_CMD_LOAD_DEC          (uint8_t)0x04 // decrease intensity at target load (dimmer/slower)
//#define SH_CMD_LOAD_SETFAV       (uint8_t)0x05 // Set user favorite intensity at target load (brightness/speed)
#define SH_CMD_LOAD_GOTOFAV      (uint8_t)0x05 // Set load current intensity to user favorite at target load (brightness/speed)
#define SH_CMD_LOAD_SAVEFAV      (uint8_t)0x06 // save current intensity as target load user favorite level (brightness/speed)
#define SH_CMD_LOAD_READFAV      (uint8_t)0x07 // read the saved favorite intensity at target load and xmit back to another node (brightness/speed))
#define SH_CMD_LOAD_READCRNT     (uint8_t)0x08 // read the current active intensity at target load (brightness/speed)
#define SH_CMD_LOAD_READPWR      (uint8_t)0x09 // read the current "Powered" state  at target load (ON/OFF, regardless of current intensity value)
#define SH_CMD_LOAD_TOGLPWR      (uint8_t)0x0a // read the current "Powered" state  at target load (ON/OFF, regardless of current intensity value)
#define SH_CMD_LOAD_EVNT_NOTICE  (uint8_t)0xff // decrease intensity at target load (dimmer/slower)

#define SH_POWERED_OFF (uint8_t)0x00
#define SH_POWERED_ON  (uint8_t)0x01


//#define SH_RESERVED_BYTE 0x00
#define SH_RESERVED_BYTE 'R'


// Informational things only
#define ZB_ID_COORD 0x00        // The Zigbee coordinator is always ID 0
#define ZB_ID_PAN   0x42        // The ID for the local network. Other networks are on different PAN IDs.


// Zigbee API frame stuff
#define ZB_START_DELIMITER           (uint8_t)0x7e // indicates start of a Zigbee frame
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


#define ZB_64ADDR_BCAST_HIGH (uint32_t)0x00000000 //64bit BCAST is 0x000000000000ffff
#define ZB_64ADDR_BCAST_LOW  (uint32_t)0x0000ffff //64bit BCAST is 0x000000000000ffff

#define ZB_16ADDR_BCAST (uint16_t)0xfffe   // 16addr bcast or unknown

#define ZB_BCAST_RADIUS (uint8_t)0x00
#define ZB_OPTIONS      (uint8_t)0x00


// Define a struct for the SmartHome payload portion of the Zigbee API Frame
typedef struct
{
    volatile uint16_t SHdestID;     // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint16_t SHsrcID;      // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint8_t  SHmsgType;    // 8bit Smarthome message type
    volatile uint8_t  SHcommand;    // 8bit Smarthome command
    volatile uint8_t  SHstatusH;    // Status value 2 / High Byte of 16bit Smarthome
    volatile uint8_t  SHstatusL;    // Status value 1 / Low byte of 16bbit Smarthome
    volatile uint8_t  SHstatusVal;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved1;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved2;  // 8bit Smarthome message type
    volatile uint8_t  SHpayldChksum;   // 8bit Smarthome message type
} SHpayload, *prtSHpayload;

#if 0
typedef struct
{
    volatile uint8_t  SHdestID_H;   // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint8_t  SHdestID_L;   // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint8_t  SHsrcID_H;    // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint8_t  SHsrcID_L;    // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint8_t  SHmsgType;    // 8bit Smarthome message type
    volatile uint8_t  SHcommand;    // 8bit Smarthome command
    volatile uint8_t  SHstatusH;    // High Byte of 16bit Smarthome
    volatile uint8_t  SHstatusL;    // Low byte of 16bbit Smarthome
    volatile uint8_t  SHstatusVal;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved1;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved2;  // 8bit Smarthome message type
    volatile uint8_t  SHpayldChksum;   // 8bit Smarthome message type
} ZBbufferSHpayload;
#endif

// Offsets into the SmartHome Message (Zigbee Payload/RF Data)
#define SH_MSG_OFFSET_ID_DEST_H    0
#define SH_MSG_OFFSET_ID_DEST_L    1
#define SH_MSG_OFFSET_ID_SRC_H     2
#define SH_MSG_OFFSET_ID_SRC_L     3
#define SH_MSG_OFFSET_MSG_TYPE     4
#define SH_MSG_OFFSET_CMD          5
#define SH_MSG_OFFSET_STATUS_H     6
#define SH_MSG_OFFSET_STATUS_L     7
#define SH_MSG_OFFSET_STATUS_VAL   8
#define SH_MSG_OFFSET_RESVD_1      9
#define SH_MSG_OFFSET_RESVD_2     10
#define SH_MSG_OFFSET_CHKSUM      11


// Define a struct for the Zigbee TX Request API frame
typedef struct
{
    volatile uint8_t    ZBfrmDelimiter;
    volatile uint16_t   ZBfrmLength;
    volatile uint8_t    ZBfrmType;
    volatile uint8_t    ZBfrmID;
    volatile uint32_t   ZBdaddr64High;
    volatile uint32_t   ZBdaddr64Low;
    volatile uint16_t   ZBdaddr16;
    volatile uint8_t    ZBfrmRadius;
    volatile uint8_t    ZBfrmOptions;
    volatile SHpayload  ZBfrmPayload;
    volatile uint8_t    ZBfrmChksum;;
} ZBframeTX, *prtZBframeTX;


// Define a struct for the Zigbee RX RCVD API frame
typedef struct
{
    volatile uint8_t    ZBfrmDelimiter;
    volatile uint16_t   ZBfrmLength;
    volatile uint8_t    ZBfrmType;
    volatile uint32_t   ZBsaddr64High;
    volatile uint32_t   ZBsaddr64Low;
    volatile uint16_t   ZBsaddr16;
    volatile uint8_t    ZBfrmOptions;
    volatile SHpayload  ZBfrmPayload;
    volatile uint8_t    ZBfrmChksum;;
} ZBframeRX, *prtZBframeRX;



//#define ZB_IN_FRAME_YES 1
//#define ZB_IN_FRAME_NO  0


// TODO clean up TX vs RX when are same thing (LEN bytes, ADDR64 bytes etc)
#define ZB_TX_FRM_DELMTR_BYTES   0x01
#define ZB_TX_FRM_LEN_BYTES      0x02
#define ZB_TX_FRM_TYPE_BYTES     0x01
#define ZB_TX_FRM_ID_BYTES       0x01
#define ZB_TX_FRM_DADDR64_BYTES  0x08
#define ZB_TX_FRM_DADDR16_BYTES  0x02
#define ZB_TX_FRM_BRADIUS_BYTES  0x01
#define ZB_TX_FRM_OPTIONS_BYTES  0x01
#define ZB_TX_FRM_HEADER_BYTES  (ZB_TX_FRM_DELMTR_BYTES + ZB_TX_FRM_LEN_BYTES + ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_ID_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_BRADIUS_BYTES + ZB_TX_FRM_OPTIONS_BYTES)
#define ZB_RX_FRM_HEADER_BYTES  (ZB_TX_FRM_DELMTR_BYTES + ZB_TX_FRM_LEN_BYTES + ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_OPTIONS_BYTES)

#define ZB_FRM_PAYLOAD_BYTES    12  // fixed number in this design
#define ZB_TX_FRM_PAYLOAD_BYTES ZB_FRM_PAYLOAD_BYTES
#define ZB_TX_FRM_CHKSUM_BYTES  1
#define ZB_TX_FRM_BYTES          (uint16_t)(ZB_TX_FRM_HEADER_BYTES + ZB_TX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES)
#define ZB_TX_FRM_BYTES_SH_MAX   (ZB_TX_FRM_HEADER_BYTES + ZB_TX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES) // max bytes SmartHome nodes should expect to receive in a ZB frame

#define ZB_RX_FRM_PAYLOAD_BYTES ZB_FRM_PAYLOAD_BYTES
#define ZB_RX_FRM_BYTES         (uint16_t)(ZB_RX_FRM_HEADER_BYTES + ZB_RX_FRM_PAYLOAD_BYTES + ZB_TX_FRM_CHKSUM_BYTES)
#define ZB_RX_FRM_BYTES_INT     (uint16_t)ZB_RX_FRM_BYTES

#define ZB_TX_FRM_LEN  (uint16_t)(ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_ID_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_BRADIUS_BYTES + ZB_TX_FRM_OPTIONS_BYTES + ZB_FRM_PAYLOAD_BYTES)
#define ZB_RX_FRM_LEN  (uint16_t)(ZB_TX_FRM_TYPE_BYTES + ZB_TX_FRM_DADDR64_BYTES + ZB_TX_FRM_DADDR16_BYTES + ZB_TX_FRM_OPTIONS_BYTES + ZB_FRM_PAYLOAD_BYTES)

// Common frame offsets (for both TX and RX frames)
#define ZB_FRM_OFFSET_DELMTR        0   // value at this offset MUST be 0x7e
#define ZB_FRM_OFFSET_LENH          1
#define ZB_FRM_OFFSET_LENL          2
#define ZB_FRM_OFFSET_FTYPE         3   // Zigbee frame type  

// TX Request frame offsets
#define ZB_FRM_OFFSET_FID           4   // Zigbee Frame ID
#define ZB_FRM_OFFSET_TX_DADDR64BH  5   // High 32bits of Zigbee 64bit addr
#define ZB_FRM_OFFSET_TX_DADDR64B7  5
#define ZB_FRM_OFFSET_TX_DADDR64B6  6
#define ZB_FRM_OFFSET_TX_DADDR64B5  7
#define ZB_FRM_OFFSET_TX_DADDR64B4  8
#define ZB_FRM_OFFSET_TX_DADDR64BL  9   // Low 32bits of Zigbee 64bit addr
#define ZB_FRM_OFFSET_TX_DADDR64B3  9
#define ZB_FRM_OFFSET_TX_DADDR64B2  10
#define ZB_FRM_OFFSET_TX_DADDR64B1  11
#define ZB_FRM_OFFSET_TX_DADDR64B0  12
#define ZB_FRM_OFFSET_TX_DADDR16H   13  // High Byte of Zigbee 16bit addr
#define ZB_FRM_OFFSET_TX_DADDR16L   14  // Low Byte of Zigbee 16bit addr
#define ZB_FRM_OFFSET_TX_BRADIUS    15
#define ZB_FRM_OFFSET_TX_OPTIONS    16
#define ZB_FRM_OFFSET_TX_PAYLOAD    17
#define ZB_FRM_OFFSET_TX_CHKSUM     (ZB_FRM_OFFSET_TX_PAYLOAD + ZB_TX_FRM_PAYLOAD_BYTES)

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
#define ZB_FRM_OFFSET_RX_CHKSUM     (ZB_FRM_OFFSET_RX_PAYLOAD + ZB_RX_FRM_PAYLOAD_BYTES)


class SHzigbee
{
    public:
	uint8_t    ZBinFrameRX;        // is a recognizable Zigbee frame coming in or not yet
	uint8_t    ZBnewFrameRXed;     // Has a new Zigbee frame been received/completed
	//uint8_t    ZBfrmLengthRX;
	uint8_t    newSHmsgRX;         // Has a new SmartHome message been received/completed
        SHpayload  SHmsgRX;            // SmartHome message received
        SHpayload  SHmsgTX;            // SmartHome message to send

	SHzigbee(); //constructor

	// Transmit a Zigbee TX REQ type API frame, previously prepared with SmartHome payload content
        uint8_t zbXmitAPIframe(void);

        // Prepare the TX frame message payload to be sent
        void prepareTXmsg( uint16_t prepSHdestID,     // Dest ID
                           uint16_t prepSHsrcID,      // Source ID
                           uint8_t  prepSHmsgType,    // Msg Type
                           uint8_t  prepSHcommand,    // CMD
                           uint8_t  prepSHstatusH,    // Status/StatusID High byte
                           uint8_t  prepSHstatusL,    // Status/StatusID Low byte
                           uint8_t  prepSHstatusVal   // Status value (8bit)
                         );                 

	// read back the current value of this TX message type (of SmartHome Message types)
        uint8_t getMsgTypeTX(void);
	
	// Receive (attempt to) a Zigbee RX RCVD type API frame, previously prepared with SmartHome payload content
	uint8_t zbRcvAPIframe(void);

    private:
        ZBframeTX     _myZBframeTX;                //A Zigbee TX REQ type API frame struct instance to work with
        ////prtZBframeTX  ptrMyZBframeTX = &_myZBframeTX;
        uint8_t _ZBfrmBufferTX[ZB_TX_FRM_BYTES];   // byte array buffer to dump into Serial.write()

        ZBframeRX     _myZBframeRX;                //A Zigbee RX RVCD type API frame struct instance to work with
        ////prtZBframeRX  ptrMyZBframeRX = &_myZBframeRX;
        uint8_t  _ZBfrmBufferRX[ZB_RX_FRM_BYTES];  // byte array buffer to dump into FROM Serial.read()
        uint16_t _ZBoffsetRXbuff;                  // offset index into the _ZBfrmBufferRX Zigbee Frame buffer
	uint8_t  _ZBfrmRXchkSumCalc;               // locally calculated checksum of the ZB API frame
        uint8_t  _SHmessageChksumCalc;             // locally calculated checksum of the SmartHome Message payload
//        uint8_t  _ZBinFrameRX;

	void     initXmitAPIframe(void);
        uint8_t  calcChkSum8(uint8_t ui8);
        uint8_t  calcChkSum16(uint16_t ui16);
        uint8_t  calcChkSum32(uint32_t ui32);
	void     debugPrintZBframeRX(void);

}; // end class SHzigbee

#endif
