/*
 *  NAME: unoNode
 *  DESCRIPTION: Arduino code for an Uno R3 type Arduino board, in the SmartHome lighting and ceiling fan control system for
 *               Bill Toner's Fall 2015 Embedded Systems project EN.525.743 at Johns Hopkins University, Dorsey Center
 *               with Professor Houser.
 *
 *               While Arduino preference is to do things in a very C++ style, I am a C programmer, and will
 *               write this software in more of a C style, due to scheduling concerns of working in
 *               a style that is slightly foreign to me.
 *
 *               Arduino Uno Target nodes in this system will be directly in control of the brightness of light fixtures,
 *               the speed of ceiling fans, and communicate with other portions of the SmartHome system via
 *               Zigbee wireless messages, using the Zigbee API mode TX Request and RX Received frames.
 *               Other Zigbee API frame types will be ignored. All Zigbee messages will be sent to broadcast address
 *               so that all other nodes will receive the same message.
 *
 *               Target loads, light fixtures or ceiling fans, will be driven by the Uno board via
 *               a set of ZeroCross Tail and PowerSwitch Tail 2 units to observe and control the
 *               120V AC power lines to the loads.
 *
 *               When a SmartHome command message is received, and is found to be addressed to a target load controlled
 *               by this Arduino unit, then this node will aknlowledge receipt of the command and ask for confirmation
 *               that the addressed sender did indeed send it. The addressed sender will then condirm or deny that it
 *               made the command request. Finally, this target load node will indicate completion of the command,
 *               along with a status to say it was done or was ignored. The command would be ignored if the
 *               addressed sender denied that it sent the command.
 *
 *               On completion of a command, an Event Notice message, including the completion status value,
 *               will be sent to the central Server node for logging. The Event Notice is essentially a command
 *               to the Server node, and will follow the same SmartHome messaging protocol described above.
 *
 *               The Zigbee message payload contains the SmartHome message, which is of a rigidly defined structure,
 *               and is made up of 12 bytes.
 *
 *               Byte 1 | Offset 0  | Description ...
 *               Byte 2 | Offset 1  |
 *               TODO - complete message description
 *
 *               TODO - complete commands descriptions
 *
 *               TODO -
 *
 *               NOTES: I had intended to share this code more with Arduino Due nodes, which I plan to use
 *               as the wall control nodes, placed where you would typically find traditional light switches,
 *               and perhaps replacing traditional light switches. I am finding that this is difficult,
 *               due to endian differences, and also due to differences in data packing and word alignments,
 *               particularly in my initial implementation of Zigbee transmit and receive functions.
 *
 */
// uncomment these DEBUG items to enable debug serial prints
//#define DEBUG_ZB_RX
//#define DEBUG_ZB_TX


// Select the Microcontroller type for this node unit
#define uC_TYPE_UNO   1  // 8bit AVR used for load Targets
#define uC_TYPE_MEGA  2  // 8bit AVR with larger memory and more board pins than Uno
#define uC_TYPE_DUE   3  // 32bit ARM CortexM3 used for Wall "switch" controllers
#define uCtype      uC_TYPE_UNO
//#define uCtype      uC_TYPE_MEGA
//#define uCtype      uC_TYPE_DUE


// Include Arduino standard libraries
#include "Arduino.h"

// Because Unos and Dues have different sizes for int
#include "stdint.h"

// Include 3rd party libraries (downloaded, not created by my and not part of Arduino standard)

// Include libraries newly created for this project
#include <ByteSwap.h>
//#include <SmartHome_EEPROM.h>





// tried to move into a library but didn't get it rught yet
#if 1
#ifdef uCtype uC_TYPE_UNO
// uncomment next line if want to init program a new Arduino Uno board for use
// Will ad dnumber of NodeIDs and fill in NodeInfo blocks for new and blank unit
#define INIT_UNO_EEPROM

#define UNO_SH_MAX_NODE_IDS               (uint16_t)2

#include <EEPROM.h>

// Uno has 512 Bytes of EEPROM, need 16bit int Offset to address them all
#define UNO_EEPROM_OFFSET_NUM_NODE_IDS    (uint16_t)0  // number of NodeInfo structures in this Arduino unit
#define UNO_EEPROM_OFFSET_N0_BASE         (uint16_t)1  // base offset to first NodeInfo structure
#define UNO_EEPROM_OFFSET_BS_LOC          (uint16_t)0  // Location offset into NodeInfo struct
#define UNO_EEPROM_OFFSET_BS_TYPE         (uint16_t)1  // Node Type offset into NodeInfo struct
#define UNO_EEPROM_OFFSET_BS_PIN          (uint16_t)2  // Node Type offset into NodeInfo struct
#define UNO_EEPROM_OFFSET_BS_IDH          (uint16_t)3  // 16bit High ID byte offset into NodeInfo struct
#define UNO_EEPROM_OFFSET_BS_IDL          (uint16_t)4  // 16bit low ID byte offset into NodeInfo struct
#define UNO_EEPROM_OFFSET_BS_POWERED      (uint16_t)5  // is the node powered or not? offset into NodeInfo struct. 
                                                       // This lets us save if it is on or off as well current intensity level
#define UNO_EEPROM_OFFSET_BS_CRNT_INTSTY  (uint16_t)6  // Current Intensity (brightness/speed) level offset into NodeInfo struct
#define UNO_EEPROM_OFFSET_BS_FAV_INTSTY   (uint16_t)7  // Favorite Intensity (brightness/speed) level offset into NodeInfo struct

#define UNO_EEPROM_OFFSET_N0_BASE         (uint16_t)1

#define UNO_EEPROM_OFFSET_N0_LOC          (uint16_t)1
#define UNO_EEPROM_OFFSET_N0_TYPE         (uint16_t)2
#define UNO_EEPROM_OFFSET_N0_PIN          (uint16_t)3
#define UNO_EEPROM_OFFSET_N0_IDH          (uint16_t)4
#define UNO_EEPROM_OFFSET_N0_IDL          (uint16_t)5
#define UNO_EEPROM_OFFSET_N0_POWERED      (uint16_t)6
#define UNO_EEPROM_OFFSET_N0_CRNT_INTSTY  (uint16_t)7
#define UNO_EEPROM_OFFSET_N0_FAV_INTSTY   (uint16_t)8
#define UNO_EEPROM_OFFSET_N1_LOC          (uint16_t)9
#define UNO_EEPROM_OFFSET_N1_TYPE         (uint16_t)10
#define UNO_EEPROM_OFFSET_N1_PIN          (uint16_t)11
#define UNO_EEPROM_OFFSET_N1_IDH          (uint16_t)12
#define UNO_EEPROM_OFFSET_N1_IDL          (uint16_t)13
#define UNO_EEPROM_OFFSET_N1_POWERED      (uint16_t)14
#define UNO_EEPROM_OFFSET_N1_CRNT_INTSTY  (uint16_t)15
#define UNO_EEPROM_OFFSET_N1_FAV_INTSTY   (uint16_t)16
#define UNO_EEPROM_OFFSET_NODE_BYTES      (uint8_t)8
#endif
#endif // #if 0

// use LOW and HIGH temporarily for early dev/debug with digital LED, before PWM is enabled
#define LOAD_POWERED_ON  LOW //(uint8_t)0x01
#define LOAD_POWERED_OFF HIGH //(uint8_t)0x00

// defines for Arduino digital pins from only the tumber to D and ht enumber, such as 6->D6
#define D6 6
#define D9 9


// TODO - find better place & reorganize
#define PWM_MIN_COUNT  (uint8_t)0
#define PWM_MAX_COUNT  (uint8_t)255
#define PWM_NUM_STEPS   (uint8_t)5
#define PWM_STEP_VAL   (uint8_t)50
//#define PWM_STEP_VAL   (uint8_t)(PWM_MAX_COUNT / PWM_NUM_STEPS)
#define PWM_FULL_OFF   (uint8_t)0
#define PWM_ON_20PCT  (uint8_t)(1 * (PWM_MAX_COUNT / 5))
#define PWM_ON_40PCT  (uint8_t)(2 * (PWM_MAX_COUNT / 5))
#define PWM_ON_60PCT  (uint8_t)(3 * (PWM_MAX_COUNT / 5))
#define PWM_ON_80PCT  (uint8_t)(4 * (PWM_MAX_COUNT / 5))
#define PWM_FULL_ON  (uint8_t)PWM_MAX_COUNT


#if 0
// The following LGPL 2.1+ byteswap macros slightly modified from
// http://repo-genesis3.cbi.utsa.edu/crossref/ns-sli/usr/include/bits/byteswap.h.html
// These byteswaps are needed, as 16bit ints and 32bit longs in the Zigbee message byte order are byteswapped
// compared to the Arduino's requirements to conveniently use them as 16bit int or 32bit long values,
// rather than doing more work to deal with everything as individual bytes
#define BYTESWAP16(x) \
  (uint16_t)((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

#define BYTESWAP32(x) \
  (uint32_t)((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |               \
             (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))
#endif


#define  NO (uint8_t)0x00
#define YES (uint8_t)0x01




// Xbee module for Zigbee must be preconfigured as a Router in API mode and 9600 8n1



#define SH_NODE_NUM_FOR_CNTRLR  1                         // For a wall switch "controller"
//#define SH_NODE_NUM_IDS  SH_NODE_NUM_FOR_CNTRLR  // For a wall switch "controller"
#define SH_NODE_NUM_IDS  2                         // How many target loads on this node?
#define SH_NODE_TYPE_SERVER  0x00  // Linux Server (NOT an Arduino platform)
#define SH_NODE_TYPE_TARGET  0x01  // Target, has Triacs/PowerTail, NO LCD
#define SH_NODE_TYPE_CNTRL   0x02  // Wall "switch" controller, has LCD, uSD

#define SH_MAX_INTENSITY 5 // Have this + 1 number of intensity levels to scroll up and down through, including 0

typedef struct
{
    volatile uint16_t SHothrID;
    volatile uint8_t  SHmsgType;
    volatile uint8_t  SHcommand;
    volatile uint8_t  SHstatusH;
    volatile uint8_t  SHstatusL;
    volatile uint16_t SHstatusID; // 16bit alternative to SHstatusH and SHstatusL but represents same bytes in message
    volatile uint8_t  SHstatusVal;
    volatile uint8_t  SHreserved1;
    volatile uint8_t  SHreserved2;
    volatile uint8_t  SHchksum;     // form this node or defined by another node
    volatile uint8_t  SHcalcChksum; // checksum calculated for comparison to sender's value
    volatile uint8_t  SHstatusTX;  // status of attempt to transmit a Zigbee API frame
    volatile uint8_t  SHstatusRX;  // status of receiving a Zigbee API frame
} SHmessage, *prtSHmessage;

typedef struct
{
    volatile uint16_t  SHthisNodeID;           // this node SH ID, might be one of multiple loads on this node
    volatile uint8_t   SHthisNodeLoc;          // what building room is this node in? (8bit room ID)
    volatile uint8_t   SHthisNodeType;         // 0=ctrl, 1=light, 2=fan
    volatile uint16_t  SHthisNodePin;          // What Arduino board pin is controlled by this load Node? This is 16bit int value such as D6, D9 etc. #defines
    volatile uint8_t   SHthisNodeIsPowered;    // is ON to some brightness/speed level, but something more than full-OFF, or full-OFF. 1=ON, 0=OFF
    volatile uint8_t   SHthisNodeLevelCurrent; // current dim/speed level for this load
    volatile uint8_t   SHthisNodeLevelFav;     // favorite dim/speed level for this load
    volatile uint16_t  SHothrNodeID;           // other node in this SH message conversation, 0 if idle
    volatile uint8_t   SHmsgCurrentState;      // which of 4 message stages are we in now, or 0=idle?
    volatile uint8_t   SHmsgNextState;         // which of 4 message stages are we in now, or 0=idle?
    volatile uint8_t   SHmsgCmd;               // current message command for this node
    volatile uint8_t   SHmsgStatus;            // current message status for this node
    SHmessage          SHthisNodeMsg;          // message fields for this node ID, to be in parallel to message fields of other Node IDs
} SHnodeInfo, *prtSHnodeInfo;

typedef struct
{
  volatile uint8_t numNodeIDs; // SH_NODE_NUM_IDS
  SHnodeInfo       nodeInfo[SH_NODE_NUM_IDS];
} SHnodeMasterInfo, *prtSHnodeMasterInfo;

SHnodeMasterInfo mySHnodeMasterInfo;


// SH message protocol stages / states
#define SH_MSG_ST_IDLE      (uint8_t)0x00  // waiting to receive a command message
#define SH_MSG_ST_CMD_INIT  (uint8_t)0x01  // begin an SH protocol message conversation
#define SH_MSG_ST_ACK_REQ   (uint8_t)0x02  // achnowledge receipt of a CMD_INIT message, request confirmation
#define SH_MSG_ST_CNFRM     (uint8_t)0x03  // confirm that this node sent a CMD_INIT message to indicated destination
#define SH_MSG_ST_COMPLETE  (uint8_t)0x04  // complete the SH protocol conversation, include status value        

// use node-specific versions of these instead, in the nodeInfo msg struct
//uint8_t currentMsgState = SH_MSG_ST_IDLE;
//uint8_t nextMsgState    = SH_MSG_ST_IDLE;

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
#define SH_CMD_LOAD_SETFAV       (uint8_t)0x05 // Set user favorite intensity at target load (brightness/speed)
#define SH_CMD_LOAD_SAVEFAV      (uint8_t)0x06 // save current intensity as target load user favorite level (brightness/speed)
#define SH_CMD_LOAD_READFAV      (uint8_t)0x07 // read the saved favorite intensity at target load and xmit back to another node (brightness/speed))
#define SH_CMD_LOAD_READCRNT     (uint8_t)0x08 // read the current active intensity at target load (brightness/speed)
#define SH_CMD_LOAD_EVNT_NOTICE  (uint8_t)0xff // decrease intensity at target load (dimmer/slower)



#define SH_NODE_ID_NUM_BYTES 2
//#define SH_RESERVED_BYTE 0x00
#define SH_RESERVED_BYTE 'R'

// Smarthome message payload variables
uint8_t SH_TargetLoadID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // destination addr
uint8_t SH_DestID[SH_NODE_ID_NUM_BYTES] = {'d', 'u'}; // destination node ID
uint8_t SH_SourceID[SH_NODE_ID_NUM_BYTES] = {'d', 'e'}; // source node ID
uint8_t SH_StatusID[SH_NODE_ID_NUM_BYTES] = {0x00, 0x00}; // node addr for Status
uint8_t SH_command = SH_CMD_NOP;
uint8_t SH_nodeStatus = 0x00;
uint8_t SH_msgCRC = 0x00;


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


uint8_t ZBframeIDnumTX = 0;  // 0 to 255, this is allowed to roll over to 0 again
uint8_t ZBframeIDnumRX = 0;  // 0 to 255, this is allowed to roll over to 0 again

// TODO - cleanup what's not used here:
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

// Common frame offsets (for both TX and RX frames)
#define ZB_FRM_OFFSET_DELMTR     0   // value at this offset MUST be 0x7e
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
//#define ZB_FRM_OFFSET_TX_CHKSUM     (ZB_FRM_OFFSET_TX_PAYLOAD + ZB_FRM_PAYLOAD_BYTES)
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


// Define a struct for the SmartHome payload portion of the Zigbee API Frame
typedef struct
{
    volatile uint16_t SHdestID;     // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint16_t SHsrcID;      // 16bit Smarthome node ID (inside payload so that it can be encrypted)
    volatile uint8_t  SHmsgType;    // 8bit Smarthome message type
    volatile uint8_t  SHcommand;    // 8bit Smarthome command
    volatile uint8_t  SHstatusH;    // High Byte of 16bit Smarthome
    volatile uint8_t  SHstatusL;    // Low byte of 16bbit Smarthome
    volatile uint8_t  SHstatusVal;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved1;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved2;  // 8bit Smarthome message type
    volatile uint8_t  SHpayldCRC;   // 8bit Smarthome message type
} SHpayload, *prtSHpayload;

//TODO - change these global vars to another instance of SHpayload struct for TX and again for TX
volatile uint16_t SHdestIDrx = 0;
volatile uint16_t SHsrcIDrx = 0;
volatile uint8_t  SHmsgTypeRX = 0;
volatile uint8_t  SHcommandRX = 0;
volatile uint16_t SHstatusIDrx = 0; //16bits
volatile uint8_t  SHstatusHrx = 0; // 8bits High of SHstatusIDrx
volatile uint8_t  SHstatusLrx = 0; // 8bits Low of SHstatusIDrx
volatile uint8_t  SHstatusValRX = 0;
volatile uint8_t  SHchksumRX = 0;
volatile uint8_t  SHreserved1rx = 0;
volatile uint8_t  SHreserved2rx = 0;

//TODO - change these global vars to another instance of SHpayload struct for TX and again for TX
volatile uint16_t SHdestIDtx = 0;
volatile uint16_t SHsrcIDtx = 0;
volatile uint8_t  SHmsgTypeTX = 0;
volatile uint8_t  SHcommandTX = 0;
volatile uint16_t SHstatusIDtx = 0; //16bits
volatile uint8_t  SHstatusHtx = 0; // 8bits High of SHstatusIDrx
volatile uint8_t  SHstatusLtx = 0; // 8bits Low of SHstatusIDrx
volatile uint8_t  SHstatusValTX = 0;
volatile uint8_t  SHchksumTX = 0;
volatile uint8_t  SHreserved1tx = 0;
volatile uint8_t  SHreserved2tx = 0;


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
    //volatile uint8_t    ZBfrmPayload[ZB_FRM_PAYLOAD_BYTES];
    volatile SHpayload  ZBfrmPayload;
    //SHpayload         ZBfrmPayload;
    volatile uint8_t    ZBfrmChksum;;
} ZBframeTX, *prtZBframeTX;

ZBframeTX     myZBframeTX;
prtZBframeTX  ptrMyZBframeTX = &myZBframeTX;
uint8_t      *txBuffer = (uint8_t *)&myZBframeTX; //get a pointer to byte which points to our Zigbee Frame stuct, to treat it as TX buffer for Serial.write
uint8_t       debugBufferTX[ZB_TX_FRM_BYTES];

// define a struct for the Zigbee RX Received API frame
typedef struct
{
    volatile uint8_t   ZBfrmDelimiter;
    volatile uint16_t  ZBfrmLength;
    volatile uint8_t   ZBfrmType;
    volatile uint32_t  ZBsaddr64High;
    volatile uint32_t  ZBsaddr64Low;
    volatile uint16_t  ZBsaddr16;
    volatile uint8_t   ZBfrmOptions;
    //volatile uint8_t   ZBfrmPayload[ZB_FRM_PAYLOAD_BYTES];
    SHpayload          ZBfrmPayload;
    volatile uint8_t   ZBfrmChksum;;
} ZBframeRX, *prtZBframeRX;

ZBframeRX     myZBframeRX; //global variable to use
prtZBframeRX  ptrMyZBframeRX = &myZBframeRX; //global pointer to use
uint8_t      *rxBuffer = (uint8_t *)&myZBframeRX; //get a pointer to byte (ie. byte array) which points to our Zigbee Frame stuct, to treat it as RX buffer for Serial.read

#define ZB_IN_FRAME_YES 1
#define ZB_IN_FRAME_NO  0

uint8_t ZBinFrameTX = ZB_IN_FRAME_NO;
uint8_t ZBoffsetTXbuff = 0; // byte counter, delimiter byte is number 0. Used as offset into txBuffer while creating a frame to transmit

uint8_t ZBinFrameRX = ZB_IN_FRAME_NO;
uint8_t ZBoffsetRXbuff = 0; // byte counter, delimiter byte is number 0. Used as offset into rxBuffer while receiving a frame

uint8_t newFrameRXed = NO;   // have received a new RX Received frame, waiting for processing
uint8_t newFrameRXstatus = 0;  // 1 if success (got full frame and ZB chksum matches our calc)
uint8_t newFrameForTX = 0;  // have a new TX Request frame ready to send out on uart/serial port

uint16_t ZBfrmLengthRX = 0;
uint8_t  ZBfrmRXchksumCalc = 0;

uint8_t  newSHmsgRX = NO; // was a new SmartHome message received via Zigbee?

#define  EVENT_NO_INPUT (uint8_t)0x00
uint8_t  userInputEvent = EVENT_NO_INPUT; // status for user input events (touchscreen/buttons etc. 1bit per possible event)


uint8_t i = 0; //for loop counter

// standard pin13 LED on Arduino Uno and Due for testing and debug. NOT compatible with LCS panel installed on Due nodes.
uint16_t ledPin = 13;                 // LED connected to digital pin 13 for debug
uint8_t ledPinState = 0;

// input pins
#define PIN_BUTTON_UP         HIGH
#define PIN_BUTTON_DOWN       LOW
#define PIN_ON_OFF            (uint16_t)A0  //Grey
#define PIN_ON                (uint16_t)A0  //Grey
#define PIN_BRIGHER           (uint16_t)A1  //Green
#define PIN_DIMMER            (uint16_t)A2  //Red
#define PIN_AC_ZERO_CROSS     (uint16_t)A4  //Blue
#define PIN_CHANGE_LOAD       (uint16_t)A4  //Blue
#define PIN_XBCONFIG          (uint16_t)A5  //Yellow
#define PIN_OFF               (uint16_t)A5  //Yellow
#define AC0CROSS_AT_CROSSING  LOW
#define AC0CROSS_NO_CROSSING  HIGH

// output pins
#define PIN_LED_ON        LOW
#define PIN_LED_OFF       HIGH
#define PIN_LED_XBCONFIG  (uint16_t)A3
#define PIN_CTRL_LIGHT   (uint16_t)6 //D6
#define PIN_CTRL_FAN      (uint16_t)5 //D5  // was 9  //D9
#define PIN_LED_RED       PIN_LED_XBCONFIG
#define PIN_LED_BLUE      PIN_CTRL_LIGHT
#define PIN_LED_GREEN     PIN_CTRL_FAN

// DEBUG pushbuttons while developing/debugging parts of this code
volatile uint8_t buttonOnOffPrev      = PIN_BUTTON_UP;
volatile uint8_t buttonOnPrev         = PIN_BUTTON_UP;
volatile uint8_t buttonOffPrev        = PIN_BUTTON_UP;
volatile uint8_t buttonBrighterPrev   = PIN_BUTTON_UP;
volatile uint8_t buttonDimmerPrev     = PIN_BUTTON_UP;
volatile uint8_t buttonChangeLoadPrev = PIN_BUTTON_UP;

// A real button that can remain in production, to aid in programming the Xbee module
volatile uint8_t buttonXBconfigPrev   = PIN_BUTTON_UP;
volatile uint8_t inXbeeConfigMode     = NO;

// Trigger signal from ZeroCross Tail unit for AC line sinewave timing of PWM
volatile uint8_t AC0CrossPrev         = AC0CROSS_NO_CROSSING;


#define NODE_INFO_INDEX_LIGHT 0  // the 0th index into NodeInfo array is for the light
#define NODE_INFO_INDEX_FAN   1  // the 1st index into NodeInfo array is for the fan
volatile uint8_t currentNodeInfoIndex= NODE_INFO_INDEX_LIGHT;  // 
//volatile uint8_t currentNodeInfoIndex= NODE_INFO_INDEX_FAN;  // 


// Initialize things at boot time, before starting the main program loop below
void setup() {
  uint8_t tmpNumNodes;

  // if need to program a blank Arduino Uno node for first use in this system
#ifdef INIT_UNO_EEPROM
  initEEPROMnodeInfo();
#endif

  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
  ledPinState = 0;

    // configure PWM things for load intensity control
    setupPWM();

    //pcintSetup(PIN_AC_ZERO_CROSS);
    setupPCint();
    
  //initialize Zigbee frame tracking
  ZBinFrameRX = ZB_IN_FRAME_NO;
  newFrameRXed = NO;
  newFrameForTX = NO;

  newSHmsgRX = NO;
  ZBfrmLengthRX = 0;
  ZBfrmRXchksumCalc = 0;

  userInputEvent = EVENT_NO_INPUT;   // start out with no user input event to handle

  // Xbee should be preconfigured for API mode, 9600, 8n1, so match that in Arduino serial port
  Serial.begin(9600);

  Serial.println("Testing, 1, 2, 3, testing");
  
  initNodeInfoUno();

  //experimenting with sending an ON command message frame TOTO - remove experiment
  //zbXmitAPIframe();
}


// main program loop, iterate infinitely until/unless hit a hard exit
void loop() {
  uint8_t nodeIDnum = 0;

  // put your main code here, to run repeatedly:

    if(inXbeeConfigMode== YES)
    {
        // NOT in Xbee module config mode, so run the SmartHome program
        
        for (nodeIDnum = 0; nodeIDnum < mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)
        {
            // check current state, and maybe do something for this node ID if pending
            doNodeIDmsgSM(nodeIDnum);
        }

        // Check if already have received a new SmartHome message waiting to be processed
        if (newSHmsgRX == NO)
        {
            //check for a new incoming frame data
            zbRcvAPIframe();
        }

        // Checkif have a new SmartHome message waiting to be sent
        if (newFrameForTX == YES)
        {
            // transmit the new TX frame
            zbXmitAPIframe();
        }

        if (userInputEvent != EVENT_NO_INPUT)
        {
            // Have some user input event from LCD touchscreen or buttons
            // determine if need to send a command
            // determine if need to update LCD display
        }
    }
}


// Read in nodeInfo data from NV storage into RAM Global Vars for use
// Uno nodes will use EEPROM for NV storage of node info, Due nodes will use microSD card
// Uno has 512 bytes of EEPROM, each at its own address
// Each Uno EEPROM address is 16bit int
uint8_t initNodeInfoUno(void)
{
  uint16_t eepromOffsetNodeBase = 0;
  uint16_t tempNodeID = 0;

  // how many node IDs are in this Arduino unit?
  mySHnodeMasterInfo.numNodeIDs = EEPROM.read(UNO_EEPROM_OFFSET_NUM_NODE_IDS);


  uint8_t th = 0;
  uint8_t tl = 0;
  uint16_t t = 0;

  // loop over number of Node IDs in this unit, initializing the info struct for each from EEPROM and default values
  for (i = 0; i < mySHnodeMasterInfo.numNodeIDs; i++)
  {
    eepromOffsetNodeBase = UNO_EEPROM_OFFSET_N0_BASE + (i * UNO_EEPROM_OFFSET_NODE_BYTES);

#if 0
    th = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_IDH);
    tl = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_IDL);
    t = ((th << 8) | tl);
    Serial.print("<th=0x");
    Serial.print(th, HEX);
    Serial.print("> <tl=0x");
    Serial.print(tl, HEX);
    Serial.print("> <t=0x");
    Serial.print(t, HEX);
    Serial.print("=");
    Serial.print(t, DEC);
    Serial.println(">");
#endif

    mySHnodeMasterInfo.nodeInfo[i].SHthisNodeID = ((EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_IDH) << 8) |
        EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_IDL)); // TODO - read from SD or EEPROM
#if 1
    Serial.print("myNodeInfo.SHthisNodeID[");
    Serial.print(i, DEC);
    Serial.print("] = ");
    Serial.println(mySHnodeMasterInfo.nodeInfo[i].SHthisNodeID, HEX);
#endif

    mySHnodeMasterInfo.nodeInfo[i].SHthisNodeLoc          = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_LOC);
    mySHnodeMasterInfo.nodeInfo[i].SHthisNodeType         = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_TYPE);
    mySHnodeMasterInfo.nodeInfo[i].SHthisNodePin          = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_PIN);
    mySHnodeMasterInfo.nodeInfo[i].SHthisNodeLevelFav     = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_FAV_INTSTY);
    mySHnodeMasterInfo.nodeInfo[i].SHthisNodeLevelCurrent = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY);
    mySHnodeMasterInfo.nodeInfo[i].SHthisNodeIsPowered    = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_POWERED);

//            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = NO;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), NO );

    mySHnodeMasterInfo.nodeInfo[i].SHmsgCurrentState = SH_MSG_ST_IDLE;
    mySHnodeMasterInfo.nodeInfo[i].SHmsgNextState    = SH_MSG_ST_IDLE;
    mySHnodeMasterInfo.nodeInfo[i].SHmsgCmd          = (uint8_t)SH_CMD_NOP;
    mySHnodeMasterInfo.nodeInfo[i].SHmsgStatus       = (uint8_t)SH_STATUS_FAILED;   //init to failed by default, make code set to success

//  UNO_EEPROM_OFFSET_BS_POWERED

    Serial.print("Initted nodeID=");
    Serial.print(mySHnodeMasterInfo.nodeInfo[i].SHthisNodeID, HEX);
    Serial.println("");
  }

  return (1);
}


// tried to move into a library but didn't get it rught yet
#if 1
// Add a new node to NodeInfo structure
// Future Use - needs enhancement to find an empty nodeInfo slot to use
void addEEPROMnodeInfo(uint8_t  nodeNumber,         // first node is num 0, second is num 1, etc
                       uint8_t  nodeLocation,
                       uint8_t  nodeType,
                       uint16_t nodePin,            // which Arduino board pin does this node drive to control its load? (D6, D9, etc)
                       uint16_t nodeID,             // 16bit int
                       uint8_t  powered,            // is power applied or not 1=yes, 0=no
                       uint8_t  currentIntensity,
                       uint8_t  favoriteIntensity )
{
  //mySHnodeMasterInfo.nodeInfo[i].SHthisNodePin          = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_PIN);
  
  if (mySHnodeMasterInfo.numNodeIDs < UNO_SH_MAX_NODE_IDS)
  {
    programEEPROMnodeInfo(nodeNumber, nodeLocation, nodeType, nodePin, nodeID, powered, currentIntensity, favoriteIntensity);
    mySHnodeMasterInfo.numNodeIDs += 1;
    EEPROM.update( UNO_EEPROM_OFFSET_NUM_NODE_IDS, mySHnodeMasterInfo.numNodeIDs );
  }
}


// Initialize/program the given EEPROM node info into EEPROM for first use of this node unit
// Only used when running an EEPROM init on a new Arduino. Not used in-system at this time.
// (later could potentially have new command messages update node ID info, but not this semester
// for now, node IDs etc. will be defined at compile and firmware upload time.
void initEEPROMnodeInfo(void)
{
  uint16_t tempNodeID = 0;

  Serial.println("Initting EEPROM values first time");

//  EEPROM.update(UNO_EEPROM_OFFSET_NUM_NODE_IDS, uint8_t(2) );
  EEPROM.update(UNO_EEPROM_OFFSET_NUM_NODE_IDS, uint8_t(2) );

  tempNodeID = 0x0709;
  programEEPROMnodeInfo( 0, 0, SH_NODE_TYPE_TARGET, PIN_CTRL_LIGHT, tempNodeID, LOAD_POWERED_OFF, 2, SH_MAX_INTENSITY );

  tempNodeID = 0x0a0c;
  programEEPROMnodeInfo( 1, 0, SH_NODE_TYPE_TARGET, PIN_CTRL_FAN, tempNodeID, LOAD_POWERED_OFF, 8, SH_MAX_INTENSITY );

  Serial.println("Completed Initting EEPROM values");
}


// program the given EEPROM node info into EEPROM
void programEEPROMnodeInfo(uint8_t  nodeNumber,         // first node is num 0, second is num 1, etc
                           uint8_t  nodeLocation,
                           uint8_t  nodeType,
                           uint16_t nodePin,            // which Arduino board Pin does this node control
                           uint16_t nodeID,             // 16bit int
                           uint8_t  powered,            // on or off, even if dimmed/slowed by current intensity value
                           uint8_t  currentIntensity,
                           uint8_t  favoriteIntensity )
{
  uint16_t eepromOffsetNodeBase = UNO_EEPROM_OFFSET_N0_BASE + (nodeNumber * UNO_EEPROM_OFFSET_NODE_BYTES);

  Serial.println("Programming EEPROM values");

  //init EEPROM values (later could potentially have new command messages update node ID info, but not this semester
  //for now, node IDs etc. will be defined at compile and firmware upload time.
  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_LOC), nodeLocation );
  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_TYPE), nodeType );
  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_PIN), nodePin );

  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_IDH), uint8_t(nodeID >> 8) );
  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_IDL), uint8_t(nodeID & 0x00ff) );

  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_POWERED), powered );
  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), currentIntensity );
  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_FAV_INTSTY), favoriteIntensity );
  EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), NO );

  Serial.println("Completed programming EEPROM values");
}
#endif

// Transmit a Zigbee API TX Request Frame
// Each Zigbee message frame has a 12byte payload, so length=23=0x17 bytes, total frame=27bytes
// FIXME
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
  //old way when initially getting frames to work. Now use prepareTXmsg() instead, need to cleanup - TODO
  //populateTXpayload();

  // initialize the Zigbee API frame checksum before calculating it
  //    myZBframeTX.ZBfrmChksum = 0x00;
  //    zbFrmCalcTxChksum();
  myZBframeTX.ZBfrmChksum = zbFrmCalcTxChksum();

  clearDebugBufferTX();
  debugPrintTxBuffer();

  for (ZBoffsetTXbuff = 0; ZBoffsetTXbuff < ZB_TX_FRM_BYTES; ZBoffsetTXbuff++)
  {
    // TODO change to if > 0 so match the receive bytes style
    while ( Serial.availableForWrite() == 0 )
    {
      //wait for room in serial buffer, do nothing while waiting
    }
    Serial.write(txBuffer[ZBoffsetTXbuff]); //Uno
    debugBufferTX[ZBoffsetTXbuff] = txBuffer[ZBoffsetTXbuff];
  }

  Serial.print("Finished sending TXframe");
  printDebugBufferTX();
}


// TODO - cleanup, replaced this one with prepareTXmsg
void populateTXpayload(void)
{
  // DEBUG data for testing the transmit
  SHdestIDtx = BYTESWAP16(0x0102);
  SHsrcIDtx = BYTESWAP16(0x0304);
  SHmsgTypeTX = SH_MSG_TYPE_CMD_REQ;
  SHcommandTX = SH_CMD_LOAD_ON;
  SHstatusHtx = 0x07;
  SHstatusLtx = 0x08;
  //SHstatusIDtx = BYTESWAP16( (uint16_t)myZBframeRX.ZBfrmPayload.SHstatusH ); //uint16 gets both SHstatusH and SHstatusL
  SHstatusIDtx = (SHstatusHtx << 8) | SHstatusLtx; //uint16 gets both SHstatusH and SHstatusL
  SHstatusValTX = 0x09;
  SHchksumTX = 0x0a;
  SHreserved1tx = 'T';
  SHreserved2tx = 'X';

  myZBframeTX.ZBfrmPayload.SHdestID = SHdestIDtx;
  myZBframeTX.ZBfrmPayload.SHsrcID = SHsrcIDtx;
  myZBframeTX.ZBfrmPayload.SHmsgType = SHmsgTypeTX;
  myZBframeTX.ZBfrmPayload.SHcommand = SHcommandTX;
  myZBframeTX.ZBfrmPayload.SHstatusH = SHstatusHtx;
  myZBframeTX.ZBfrmPayload.SHstatusL = SHstatusLtx;
  myZBframeTX.ZBfrmPayload.SHstatusVal = SHstatusValTX;
  myZBframeTX.ZBfrmPayload.SHpayldCRC = SHchksumTX;
  myZBframeTX.ZBfrmPayload.SHreserved1 = SHreserved1tx;
  myZBframeTX.ZBfrmPayload.SHreserved2 = SHreserved2tx;
}


// Prepare the TX frame message payload to be sent
void prepareTXmsg( uint16_t prepSHsrcID,      // Source ID
                   uint16_t prepSHdestID,     // Dest ID
                   uint8_t  prepSHmsgType,    // Msg Type
                   uint8_t  prepSHcommand,    // CMD
                   uint8_t  prepSHstatusH,
                   uint8_t  prepSHstatusL,
                   uint8_t  prepSHstatusVal
                 )
{
  myZBframeTX.ZBfrmPayload.SHdestID    = BYTESWAP16(prepSHdestID);
  myZBframeTX.ZBfrmPayload.SHsrcID     = BYTESWAP16(prepSHsrcID);
  myZBframeTX.ZBfrmPayload.SHmsgType   = prepSHmsgType;
  myZBframeTX.ZBfrmPayload.SHcommand   = prepSHcommand;
  myZBframeTX.ZBfrmPayload.SHstatusH   = prepSHstatusH;
  myZBframeTX.ZBfrmPayload.SHstatusL   = prepSHstatusL;
  myZBframeTX.ZBfrmPayload.SHstatusVal = prepSHstatusVal;
  myZBframeTX.ZBfrmPayload.SHreserved1 = (uint8_t)'T';
  myZBframeTX.ZBfrmPayload.SHreserved2 = (uint8_t)'X';
  //calc payload message checksum
  myZBframeTX.ZBfrmPayload.SHpayldCRC  = doCalcMsgChecksum(ZB_FRAME_TYPE_TX_REQ);
}


// calculate checksum for the given SH message payload structure inside of the Zigbee API frame
// NOT the Zigbee Frame checksum value as last byte of the API frame
//uint8_t doCalcMsgChecksum(SHpayload msgToCalc)
uint8_t doCalcMsgChecksum(uint8_t ZBframeType)
{
  uint8_t calcChecksum = 0;

  if (ZBframeType == ZB_FRAME_TYPE_TX_REQ)
  {
    // payloadBytes-1 so do not checksum the message checksum value
    for (i = ZB_FRM_OFFSET_TX_PAYLOAD; i < (ZB_FRM_OFFSET_TX_PAYLOAD + ZB_TX_FRM_PAYLOAD_BYTES - 1); i++)
    {
      calcChecksum += txBuffer[i];
    }
  }
  else if (ZBframeType == ZB_FRAME_TYPE_RX_RCVD)
  {
    // payloadBytes-1 so do not checksum the sender's message checksum value
    for (i = ZB_FRM_OFFSET_RX_PAYLOAD; i < (ZB_FRM_OFFSET_RX_PAYLOAD + ZB_RX_FRM_PAYLOAD_BYTES - 1); i++)
    {
      calcChecksum += rxBuffer[i];
#if 0
      Serial.print(" <rxB=");
      Serial.print(rxBuffer[i], HEX);
      Serial.print(" ; cs=");
      Serial.print(calcChecksum, HEX);
      Serial.print("> ");
#endif
    }
  }

  calcChecksum = ( (uint8_t)0xff - calcChecksum );

  return (calcChecksum);
}


// calculate the Zigbee frame checksum for this API TX Request frame
uint8_t zbFrmCalcTxChksum(void)
{
  uint8_t ZBfrmTXchksumCalc = 0;

  for (ZBoffsetTXbuff = ZB_FRM_OFFSET_FTYPE; ZBoffsetTXbuff < ZB_FRM_OFFSET_TX_CHKSUM; ZBoffsetTXbuff++)
  {
    // add to the checksum for this frame
    ZBfrmTXchksumCalc += txBuffer[ZBoffsetTXbuff];
  }

  // Final part of Zigbee frame checksum calculation is to subtract current total from 0xff
  // Save into txbuffer for sending
  ZBfrmTXchksumCalc = 0xff - ZBfrmTXchksumCalc;


#if 0
  Serial.print(" <<zbTxChksum=");
  Serial.print(ZBfrmTXchksumCalc, HEX);
  Serial.print(" ?=? ");
  Serial.print(txBuffer[ZB_FRM_OFFSET_TX_CHKSUM], HEX);
  Serial.print(">> ");
#endif

  return (ZBfrmTXchksumCalc);
}


void clearDebugBufferTX(void)
{
  for (i = 0; i < ZB_TX_FRM_BYTES; i++)
  {
    debugBufferTX[i] = (uint8_t)0x00;
  }
}


void printDebugBufferTX(void)
{
  Serial.println("");
  Serial.print("debugBufferTX -> ");
  for (i = 0; i < ZB_TX_FRM_BYTES; i++)
  {
    Serial.print(debugBufferTX[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

void debugPrintTxBuffer(void)
{
  Serial.println("");
  Serial.print("Ready to send a Zigbee API TX Request frame buffer -> ");
  //for(i=0; i<=ZB_FRM_OFFSET_TX_CHKSUM; i++)
  for (i = 0; i < ZB_TX_FRM_BYTES; i++)
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
  uint8_t ZBchksumFromSender = 0;

  while (Serial.available() > 0)
  {
    // Read a byte from uart RX buffer
    ZB_frm_byte = Serial.read();

    if ((ZBinFrameRX == ZB_IN_FRAME_NO) && (ZB_frm_byte == ZB_START_DELIMITER))
    {
      // beginning a new frame
      ZBinFrameRX = ZB_IN_FRAME_YES;
      ZBoffsetRXbuff = 0; //Delimiter byte is offset 0 into RX buffer
      ZBfrmRXchksumCalc = 0; //new frame starts new checksum
    }
    else if (ZBinFrameRX == ZB_IN_FRAME_NO) // and new byte, which is NOT ZB_START_DELIMITER
    {
      // NOT already in a frame, and NOT starting a new frame here, ignore unknown bytes in RX
      return (0);
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


#if 0
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
      if ( (ZBfrmLengthRX == ZB_RX_FRM_LEN) &&
           (ZBchksumFromSender == ZBfrmRXchksumCalc) )
      {
        //Serial.print(" <Ding1> ");
        // Have a valid Zigbee frame of our expected length, assume it is valid SH message

        // pull out our SmartHome data items into global vars
        extractRXpayload();

        // let rest of program know that a new RX message is waiting to be processed
        newSHmsgRX = YES;
      }

      // end of frame, next byte received is NOT part of this same frame
      ZBinFrameRX = ZB_IN_FRAME_NO;
      return (0);
    }


    if (ZBinFrameRX == ZB_IN_FRAME_YES)
    {
      //put received byte into rx buffer (aka received Zigbee frame structure)
      rxBuffer[ZBoffsetRXbuff] = ZB_frm_byte;

      if (ZBoffsetRXbuff == ZB_FRM_OFFSET_LENL)
      {
        // have all bytes of the Zigbee Frame Length field, put this into a uint16_t variable
        ZBfrmLengthRX = BYTESWAP16(myZBframeRX.ZBfrmLength);
      }
      else if ((ZBoffsetRXbuff >= ZB_FRM_OFFSET_FTYPE) && (ZBoffsetRXbuff < ZB_FRM_OFFSET_RX_CHKSUM))
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
  // TODO - sort out duplication between these two styles, or if both styles should remain for "double-buffering"
  SHdestIDrx = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHdestID);
  SHsrcIDrx = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHsrcID);
  SHmsgTypeRX = myZBframeRX.ZBfrmPayload.SHmsgType;
  SHcommandRX = myZBframeRX.ZBfrmPayload.SHcommand;
  SHstatusHrx = myZBframeRX.ZBfrmPayload.SHstatusH;
  SHstatusLrx = myZBframeRX.ZBfrmPayload.SHstatusL;
  SHstatusIDrx = (SHstatusHrx << 8) | SHstatusLrx; //uint16 gets both SHstatusH and SHstatusL
  SHstatusValRX = myZBframeRX.ZBfrmPayload.SHstatusVal;
  SHchksumRX = myZBframeRX.ZBfrmPayload.SHpayldCRC;
  SHreserved1rx = myZBframeRX.ZBfrmPayload.SHreserved1;
  SHreserved2rx = myZBframeRX.ZBfrmPayload.SHreserved2;

#if 0
  uint8_t nodeInfoIndex = 0; // 0xff means not in this unit

  // find node ID struct matching the destination node address, and fill in the mssage fields for that node ID
  //SHdestIDrx = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHdestID);
  for (nodeInfoIndex = 0; nodeInfoIndex < mySHnodeMasterInfo.numNodeIDs; nodeInfoIndex++)
  {
    //    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgCurrentState = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState;
    if (SHdestIDrx == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID)
    {
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID    = BYTESWAP16(myZBframeRX.ZBfrmPayload.SHsrcID);
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType   = myZBframeRX.ZBfrmPayload.SHmsgType;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand   = myZBframeRX.ZBfrmPayload.SHcommand;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH   = myZBframeRX.ZBfrmPayload.SHstatusH;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL   = myZBframeRX.ZBfrmPayload.SHstatusL;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusID  = ((myZBframeRX.ZBfrmPayload.SHstatusH << 8) | myZBframeRX.ZBfrmPayload.SHstatusL);
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = myZBframeRX.ZBfrmPayload.SHstatusVal;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved1 = myZBframeRX.ZBfrmPayload.SHreserved1;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved2 = myZBframeRX.ZBfrmPayload.SHreserved2;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHchksum    = myZBframeRX.ZBfrmPayload.SHpayldCRC;
      //SHcalcChksum
    }
    // else this SH message was to a different node, do nothing
  }
#endif
}


// parse the received ZB frame payload to determine if message was for this node or not
// and if it was, determine what to do as result
// TODO - is this still used?
void processSHmsg(void)
{
  uint8_t thisNodeIndex = (uint8_t)0xff; // 0xff means not in this unit
  //debugPrintRxBuffer();


  //check if SHdestIDrx is this node cotnroller or one of its target loads
  //go through dest IDs stored in SD/eeprom (NV) and compare with SHdestIDrx in message
  for (i = 0; i < mySHnodeMasterInfo.numNodeIDs; i++)
  {
    if (SHdestIDrx == mySHnodeMasterInfo.nodeInfo[i].SHthisNodeID)
    {
      //goober
      // run the received SH command
      SHrunCommand(i);
    }
    // else this SH message was to a different node, do nothing
  }

  // We processed our new RX frame, so no longer have a new one
  // Last thing to do when processing an RX frame, so we're ready to receive another new frame
  newSHmsgRX = NO;
}


void doNodeIDmsgSM(uint8_t nodeInfoIndex)
{
  volatile uint8_t   SHmsgNextState;         // which of 4 message stages are we in now, or 0=idle?
  volatile uint8_t   SHmsgCmd;
  volatile uint8_t   SHmsgStatus;

  // update to previous iteration's next state
  //currentMsgState = nextMsgState;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgCurrentState = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState;

  switch ( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgCurrentState )
  {
    // Linux Server will receive ALL messages, and log ALL messages. No need to detect errors and specifically tell server of them.

    case SH_MSG_ST_IDLE:  // until RX SH_MSG_ST_CMD_INIT
#if 0
      Serial.print("ST_IDLE, newSHmsgRX=");
      Serial.print(newSHmsgRX, HEX);
      Serial.print(" ; SHmsgTypeRX=");
      Serial.print(SHmsgTypeRX, HEX);
      Serial.print(" ; SHdestIDrx=");
      Serial.println(SHdestIDrx, HEX);
#endif
      if ( (newSHmsgRX == YES) && (SHmsgTypeRX == SH_MSG_TYPE_CMD_REQ) && (SHdestIDrx == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID) )
      {
        // TODO - going to keep it this way? - move to extractRXpayload? - keep this way for now
        captureRXmsg(nodeInfoIndex);
        Serial.print("New CMD at nodeID=");
        Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
        Serial.print(" ?=? ");
        Serial.print(SHdestIDrx, HEX);
        Serial.print(" ; from nodeID=");
        Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID, HEX);
        Serial.print(" ; CMD=");
        Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
        Serial.print(" ; chksum=");
        Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHchksum, HEX);

        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_ACK_REQ;
      }
      else // this SH message was to a different node, do nothing this iteration
      {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_IDLE;
      }
      break;

    case SH_MSG_ST_ACK_REQ:  // TX
      //            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_CNFRM;

#if 1
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusTX = TXmsgACKREQ(nodeInfoIndex);
      if ( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusTX == SH_STATUS_SUCCESS )
      {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_CNFRM;
      }
      else
      {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_IDLE;
      }
#endif

      break;

    case SH_MSG_ST_CNFRM:  // RX
      if ( (newSHmsgRX == YES) && (SHmsgTypeRX == SH_MSG_TYPE_CONFIRM) && (SHdestIDrx == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID) )
      {
        // TODO - going to keep it this way? - move to extractRXpayload? - keep this way for now
        captureRXmsg(nodeInfoIndex);

        Serial.print("nodeID=");
        Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
        Serial.print(" got confirm from ");
        Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID, HEX);
        Serial.print(" ; msgType=");
        Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType, HEX);
        Serial.print(" ; CMD=");
        Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
        Serial.print(" ; confirm=");
        Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal, HEX);

        // if confirmation is confirmed
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_COMPLETE;
        // else if confirmation is denied
        // TODO - add more states to send error notice to server
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_IDLE;
      }
      else // this SH message was to a different node, do nothing this iteration
      {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_CNFRM;
      }
      break;

    case SH_MSG_ST_COMPLETE: // TX
      // run the received SH command
      SHrunCommand(nodeInfoIndex);
      prepareTXmsg( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID,             // Src ID is this node
                    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID,   // DestID is node that initiated this conversation
                    SH_MSG_TYPE_COMPLETED,                                               // MsgType
                    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand,  // CMD
                    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH,
                    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL,
                    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal
                  );

      Serial.print("nodeID=");
      Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
      Serial.print(" sending COMPLETED to ");
      Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID, HEX);
      Serial.print(" ; msgType=");
      Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType, HEX);
      Serial.print(" ; CMD=");
      Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
      Serial.print(" ; status=");
      Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal, HEX);

      // indicate main loop that a TX frame is ready to send
      newFrameForTX = YES;
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_IDLE;
      break;

    default: // RX or TX
      // invalid next message state, reset to idle, careful about stranding another node mid-conversation.
      // TODO - timeout to help with that
      mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_IDLE;
      break;
  }
}


// Prepare a TX Zigbee frame to send a SmartHome ACK/REQ message back to the node that initiated a new command
uint8_t TXmsgACKREQ(uint8_t nodeInfoIndex)
{
  prepareTXmsg( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID,             // Src ID is this node
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID,   // DestID is node that initiated this conversation
                SH_MSG_TYPE_ACK_CREQ,                                                // MsgType
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand,  // CMD
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH,
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL,
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal
              );

  Serial.print("nodeID=");
  Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
  Serial.print(" sending ACK/REQ to ");
  Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID, HEX);
  Serial.print(" ; msgType=");
  Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType, HEX);
  Serial.print(" ; CMD=");
  Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);

  // indicate main loop that a TX frame is ready to send
  newFrameForTX = YES;
  zbXmitAPIframe();
  newFrameForTX = NO;

  return (SH_STATUS_SUCCESS);
}


// capture the received message into this Node ID's structure for processing inparallel to possible messages to another node
void captureRXmsg(uint8_t nodeInfoIndex)
{
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID    = SHsrcIDrx;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType   = SHmsgTypeRX;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand   = SHcommandRX;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH   = SHstatusHrx;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL   = SHstatusLrx;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusID  = SHstatusIDrx;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = SHstatusValRX;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved1 = SHreserved1rx;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved2 = SHreserved2rx;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHchksum    = SHchksumRX;
  mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcalcChksum    = doCalcMsgChecksum(ZB_FRAME_TYPE_RX_RCVD);
}



void SHrunCommand(uint8_t nodeInfoIndex)
{
  //switch ( SHcommandRX )
  switch ( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand )
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

  //goober
#if 0
  {
    volatile uint16_t  SHthisNodeID;       // this node SH ID, might be one of multiple loads on this node
    volatile uint8_t   SHthisNodeType;     // 0=ctrl, 1=light, 2=fan
    volatile uint8_t   SHthisNodeType;     // 0=ctrl, 1=light, 2=fan
    volatile uint8_t   SHthisNodeLevelFav;     // favorite dim/speed level for this load
    volatile uint8_t   SHthisNodeLevelCurrent; // current dim/speed level for this load
    volatile uint16_t  SHothrNodeID;       // other node in this SH message conversation, 0 if idle
    volatile uint8_t   SHmsgState;         // which of 4 message stages are we in now, or 0=idle?
    volatile uint8_t   SHmsgStatus;
  } SHnodeInfo, *prtSHnodeInfo;
#define SH_CMD_NOP               0x00 // No OPeration, do nothing
#define SH_CMD_LOAD_ON           0x01 // Turn on the target load at this node
#define SH_CMD_LOAD_OFF          0x02 // Turn off the target load at this node
#define SH_CMD_LOAD_INC          0x03 // increase intensity at target load (brighter/faster)
#define SH_CMD_LOAD_DEC          0x04 // decrease intensity at target load (dimmer/slower)
#define SH_CMD_LOAD_SETFAV       0x05 // Set user favorite intensity at target load (brightness/speed)
#define SH_CMD_LOAD_SAVEFAV      0x06 // save current intensity as target load user favorite level (brightness/speed)
#define SH_CMD_LOAD_READFAV      0x07 // read the saved favorite intensity at target load and xmit back to another node (brightness/speed))
#define SH_CMD_LOAD_READCRNT     0x08 // read the current active intensity at target load (brightness/speed)
#define SH_CMD_LOAD_EVNT_NOTICE  0xff // decrease intensity at target load (dimmer/slower)
#endif


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
  for (i = 0; i <= ZB_FRM_OFFSET_RX_CHKSUM; i++)
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
  if (ledPinState == 0)
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

#if 0
// TODO cleanup
void pcintSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


// handle pin change interrupt for AC Zero-Cross signal on A4 here
ISR (PCINT0_vect) 
{
  uint8_t ACzeroCross = 0;

  ACzeroCross = digitalRead(A4);

  if( (ACzeroCross == 0) && (ACzeroCrossPrev == 1) )
  {
      // have an AC line Zero-cross event
      triggerTriac();
  } 
}
#endif



void enablePCint(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


#define PWM_MIN_COUNT  (uint8_t)0
#define PWM_MAX_COUNT  (uint8_t)255
#define PWM_NUM_STEPS   (uint8_t)5
#define PWM_STEP_VAL   (uint8_t)50
//#define PWM_STEP_VAL   (uint8_t)(PWM_MAX_COUNT / PWM_NUM_STEPS)
#define PWM_FULL_OFF   (uint8_t)0
#define PWM_ON_20PCT  (uint8_t)(1 * (PWM_MAX_COUNT / 5))
#define PWM_ON_40PCT  (uint8_t)(2 * (PWM_MAX_COUNT / 5))
#define PWM_ON_60PCT  (uint8_t)(3 * (PWM_MAX_COUNT / 5))
#define PWM_ON_80PCT  (uint8_t)(4 * (PWM_MAX_COUNT / 5))
#define PWM_FULL_ON  (uint8_t)PWM_MAX_COUNT



// NOT YET WORKING
// turn the load on or off. If off, turn on to the previous intensity level, not necessarily 100%-on
void loadToggle(uint8_t nodeInfoIndex)
{
    if(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered == LOAD_POWERED_ON)  // Load is currently powered ON (possibly dim/slow, but ON)
    {
        // Turn it off
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered == LOAD_POWERED_OFF;
//        EEPROM.update(UNO_EEPROM_OFFSET_BS_POWERED, PIN_LED_OFF);
        digitalWrite(PIN_CTRL_LIGHT, PIN_LED_OFF);
    }
    else  // Load is currently powered full-OFF
    {
        // Turn it ON (not necessarily full-ON, may be dimmed/slow intensity level, but not full-OFF
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered == LOAD_POWERED_ON;
//        EEPROM.update(UNO_EEPROM_OFFSET_BS_POWERED, PIN_LED_ON);
        digitalWrite(PIN_CTRL_LIGHT, PIN_LED_ON);
    }

    Serial.print("DEBUG - In loadToggle(), new intensity value=");  
    Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered, DEC);
    Serial.print("for NodeInfoIndex=");
    Serial.println(nodeInfoIndex);
    
//    EEPROM.read(); // get new value just saved from EEPROM

// LOAD_POWERED_ON

// TODO - EEPROM needs another byte per load to save on/off state as well as current intensity. If turn off, store that, but also keep current intensity for next on.
    // calculate new PWM value for analogWrite() call
}

// If load is currently full-OFF, then apply power to previous current intensity level
// (ON/OFF is essentially an enable condition to the current intensity level)
void loadPowerON(uint8_t nodeInfoIndex)
{
    uint16_t tmpVal = 0;

    Serial.print("DEBUG - In loadPowerON()");    

    if( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered == NO)
    {
        tmpVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;

        // sanity check
        if( (tmpVal > PWM_MAX_COUNT) )
        {
            tmpVal = PWM_MAX_COUNT;
        }

        analogWrite(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodePin, tmpVal);
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = tmpVal;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), tmpVal );

        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = YES;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), YES );
        
        Serial.print(" ; Load ");
        Serial.print(nodeInfoIndex, DEC);
        Serial.print(" is now ON at intensity = ");
        Serial.println(tmpVal, DEC);
    }
    else
    {
        Serial.println(" ; Already ON, do nothing");      
    }
}


// If load is currently full-OFF, then apply power to previous current intensity level
// (ON/OFF is essentially an enable condition to the current intensity level)
void loadPowerOFF(uint8_t nodeInfoIndex)
{
    Serial.print("DEBUG - In loadPowerOFF()");    

    if( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered == YES)
    {
        analogWrite(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodePin, PWM_MIN_COUNT);

        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = NO;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), NO );
        
        Serial.print(" ; Load ");
        Serial.print(nodeInfoIndex, DEC);
        Serial.println(" is now OFF");
    }
    else
    {
        Serial.println(" ; Already OFF, do nothing");      
    }
}


// increase intensity level one step
void loadIncreaseIntensity(uint8_t nodeInfoIndex)
{
    uint16_t tmpVal = 0;
//    uint8_t tmpVal = 0;

//mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodePin          = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_PIN);

    Serial.print("DEBUG - In loadIncreaseIntensity()");    

    tmpVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;

    // TODO - expand this checkign to catch if new value is greater than MAX but would rollover and appear less
//    if( (tmpVal > PWM_MAX_COUNT) || ((tmpVal <= PWM_MAX_COUNT) && ((tmpVal + PWM_STEP_VAL) < tmpVal) ) )
    if( (tmpVal > PWM_MAX_COUNT) || ((tmpVal <= PWM_MAX_COUNT) && ((tmpVal + PWM_STEP_VAL) > PWM_MAX_COUNT) ) )
    {
        tmpVal = PWM_MAX_COUNT;
    }
    else
    {
        tmpVal += PWM_STEP_VAL;
    }

    analogWrite(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodePin, tmpVal);
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = tmpVal;
//    EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), tmpVal );

    Serial.print(" ; Load ");
    Serial.print(nodeInfoIndex, DEC);
    Serial.print(" new intensity = ");
    Serial.println(tmpVal, DEC);
}

// decrease intensity level one step
void loadDecreaseIntensity(uint8_t nodeInfoIndex)
{
    uint16_t tmpVal = 0;
//    uint8_t tmpVal = 0;

    Serial.print("DEBUG - In loadDecreaseIntensity()");    

//    tmpVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent - PWM_STEP_VAL;
    tmpVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;

    if( (tmpVal < PWM_MIN_COUNT) || ((tmpVal >= PWM_MIN_COUNT) && ((tmpVal - PWM_STEP_VAL) > tmpVal) ) )
    {
        tmpVal = PWM_MIN_COUNT;
    }    
    else
    {
        tmpVal -= PWM_STEP_VAL;
    }

    analogWrite(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodePin, tmpVal);
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = tmpVal;
//    EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), tmpVal );

    Serial.print(" ; Load ");
    Serial.print(nodeInfoIndex, DEC);
    Serial.print(" new intensity = ");
    Serial.println(tmpVal, DEC);
}


// Change control to the next load on this target node
void changeLoad(uint8_t nodeInfoIndex)
{
    Serial.print("DEBUG - In changeLoad()");    

    nodeInfoIndex+= 1;

    // Check if at last one
    if( nodeInfoIndex == mySHnodeMasterInfo.numNodeIDs)
    {
        // Go back to first one
        currentNodeInfoIndex = 0;
    }
    else
    {
        // Go to next one
        currentNodeInfoIndex = nodeInfoIndex;
    }

    Serial.print(" ; Now controlling Load  ");
    Serial.println(currentNodeInfoIndex, DEC);
}


// trigger the AC line triacs to fire, turning on 
void enableSSRrelay(uint8_t nodeInfoIndex)
{
    Serial.println("DEBUG - In enableSSRrelay()");    
//    analogWrite(pin, 0); // disable the PWM to the load triacs
//    TODO - add PWM output pin ID to the load's NodeInfo structure in EEPROM, for analogWrite calls
// calculate new PWM value in places that change the current intensity, and in powerOn setup function
//    analogWrite(pin, newValue); // enable 
}


// Interrupt vector handling routine for A0 to A5 Pin Change Interrupt inputs
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
#if 0  
    // check if AC Zero cross trigger OR debug Blue button
    if( (digitalRead(PIN_AC_ZERO_CROSS) == AC0CROSS_AT_CROSSING) && (AC0CrossPrev   == AC0CROSS_NO_CROSSING) )
    {
        enableSSRrelay(currentNodeInfoIndex);
    }
    else
#else
        // check if debug Blue button
        if( (digitalRead(PIN_CHANGE_LOAD) == PIN_BUTTON_DOWN) && (buttonDimmerPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(60);
            if(digitalRead(PIN_CHANGE_LOAD) == PIN_BUTTON_DOWN)
            {
                changeLoad(currentNodeInfoIndex);         
            }        
        }
#endif
    {

        // manual bushbuttons are used for early dev/debug of load control events processing.
        // Do software debounce on these pushbuttons. When move to other control methods later, 
        // comment out the pushbutton code or delete it. In final production system,
        // it is not expected to have these debounce delay statements here, which might affect the 
        // 60Hz AC Zero-Crossing trigger reliability.

#if 0
        // check if debug Grey button
        if( (digitalRead(PIN_ON_OFF) == PIN_BUTTON_DOWN) && (buttonOnOffPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(60);
            if(digitalRead(PIN_ON_OFF) == PIN_BUTTON_DOWN)
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadToggle(currentNodeInfoIndex);         
            }
        }
#else
        // check if debug Grey button
        if( (digitalRead(PIN_ON) == PIN_BUTTON_DOWN) && (buttonOnPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(60);
            if(digitalRead(PIN_ON) == PIN_BUTTON_DOWN)
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadPowerON(currentNodeInfoIndex);         
            }
        }
#endif

#if 0
        // check if debug Yellow button
        if( (digitalRead(PIN_XBCONFIG) == PIN_BUTTON_DOWN) && (buttonXBconfigPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(60);
            if(digitalRead(PIN_XBCONFIG) == PIN_BUTTON_DOWN)
            {
                toggleXbeeConfigMode();          
            }
        }
#else
        // check if debug Yellow button
        if( (digitalRead(PIN_OFF) == PIN_BUTTON_DOWN) && (buttonOffPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(60);
            if(digitalRead(PIN_OFF) == PIN_BUTTON_DOWN)
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadPowerOFF(currentNodeInfoIndex);         
            }
        }
#endif


        // check if debug Green button
        if( (digitalRead(PIN_BRIGHER) == PIN_BUTTON_DOWN) && (buttonBrighterPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(60);
            if(digitalRead(PIN_BRIGHER) == PIN_BUTTON_DOWN)
            {
                loadIncreaseIntensity(currentNodeInfoIndex);         
            }        
        }

        // check if debug Red button
        if( (digitalRead(PIN_DIMMER) == PIN_BUTTON_DOWN) && (buttonDimmerPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(60);
            if(digitalRead(PIN_DIMMER) == PIN_BUTTON_DOWN)
            {
                loadDecreaseIntensity(currentNodeInfoIndex);         
            }        
        }

    }
}  


// toggle between Xbee config mode and normal running mode.
// when Xbee Config mode is active, the AVR microcontroller should go "dormant", the SmartHome software will halt/pause execution,
// and stay off the uart serial port, so that the serial port is used to configure the Xbee module on the board using XCTU from a PC.
// When Xbee config mode is NOT active, then the SmartHome software will run normally and make use of the Xbee module for communications.
void toggleXbeeConfigMode(void)
{
    if(inXbeeConfigMode == NO)  // Xbee config mode is currently DISabled
    {
        // ENable Xbee config mode
        inXbeeConfigMode = YES;  // enable Xbee module config mode, halt SmartHome program execution
        digitalWrite(PIN_LED_XBCONFIG, PIN_LED_ON);
    }
    else  // Xbee config mode is currently ENabled
    {
        // DISable Xbee config mode
        inXbeeConfigMode= NO;  // disable Xbee module config mode, result SmartHome program execution
        digitalWrite(PIN_LED_XBCONFIG, PIN_LED_OFF);
    }
    Serial.print("DEBUG - in toggleXbeeConfigMode() new value=");
    Serial.println(inXbeeConfigMode, DEC);
}


// configure the PWM things
// PWM frequench should be 60Hz or very close to that. Just longer is preferable to just shorter than 60Hz.
void setupPWM(void)
{
    pinMode(PIN_CTRL_LIGHT,OUTPUT);  // LED PIN_LED_BLUE
    pinMode(PIN_CTRL_FAN,OUTPUT);  // LED PIN_LED_GREEN

    digitalWrite(PIN_LED_XBCONFIG, PIN_LED_OFF);
    digitalWrite(PIN_CTRL_LIGHT, PIN_LED_OFF);
    digitalWrite(PIN_CTRL_FAN, PIN_LED_OFF);

    // initially configure PWM output for the Light load, at Full-OFF
    TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz    
//    TCNT0 = 0;  // set timer counter #0 count value to 0
    pinMode(PIN_CTRL_LIGHT, OUTPUT);
    analogWrite(PIN_CTRL_LIGHT, PWM_FULL_OFF);

    // iinitially configure PWM output for the Fan load, at Full-OFF
    pinMode(PIN_CTRL_FAN, OUTPUT);
    analogWrite(PIN_CTRL_FAN, PWM_FULL_OFF);
}

// configure the Pin Change Interrupt things for detecting AC Zero Cross signal and user push buttons
void setupPCint(void) 
{  
    buttonOnOffPrev    = PIN_BUTTON_UP;
    buttonOnPrev       = PIN_BUTTON_UP;
    buttonOffPrev      = PIN_BUTTON_UP;
    buttonBrighterPrev = PIN_BUTTON_UP;
    buttonDimmerPrev   = PIN_BUTTON_UP;
    buttonXBconfigPrev = PIN_BUTTON_UP;
    buttonChangeLoadPrev = PIN_BUTTON_UP;

    AC0CrossPrev       = AC0CROSS_NO_CROSSING;
    inXbeeConfigMode   = NO;

    pinMode(PIN_LED_XBCONFIG,OUTPUT);  // LED PIN_LED_RED

    // User input pushbuttons
//    digitalWrite(PIN_ON_OFF,HIGH);         // Grey
    digitalWrite(PIN_ON,HIGH);             // Grey
    digitalWrite(PIN_BRIGHER,HIGH);        // Green
    digitalWrite(PIN_DIMMER,HIGH);         // Red
//    digitalWrite(PIN_AC_ZERO_CROSS,HIGH);  // Blue
    digitalWrite(PIN_CHANGE_LOAD,HIGH);  // Blue
//    digitalWrite(PIN_XBCONFIG,HIGH);       // Yellow
    digitalWrite(PIN_OFF,HIGH);            // Yellow

    
    // enable Pin Change Interrupt for pin...
//    enablePCint(PIN_ON_OFF);
    enablePCint(PIN_ON);
    enablePCint(PIN_BRIGHER);
    enablePCint(PIN_DIMMER);
//    enablePCint(PIN_AC_ZERO_CROSS);
    enablePCint(PIN_CHANGE_LOAD);
//    enablePCint(PIN_XBCONFIG);
    enablePCint(PIN_OFF);
}

