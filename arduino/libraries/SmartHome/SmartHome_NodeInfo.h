#ifndef DEF_SH_NODE_INFO_H
#define DEF_SH_NODE_INFO_H

/***************************************************************
 * Name:      SmartHome_NodeInfo.h
 * Purpose:   Defines structures and constant values used for SmartHome Node information,
 *            such as Zigbee network node ID, node type (light, fan, server/gateway, etc),
 *            any currently pending Zigbee network message received or to be sent, etc.
 * Author:    Bill Toner (wtoner1@jhu.edu)
 * Created:   2015-11-6
 * Copyright: Bill Toner (2015)
 * License:   This library is free software; you can redistribute it and/or
 *            modify it under the terms of the GNU Lesser General Public
 *            License as published by the Free Software Foundation; either
 *            version 3.0 of the License, or (at your option) any later version.
 *            http://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
 **************************************************************/


// SmartHome node Status Values
#define SH_STATUS_SUCCESS       (uint8_t)0x00 // will likely often use a current intensity level value instead of this
#define SH_STATUS_CONFIRMED     (uint8_t)0xfd // was 0x02
#define SH_STATUS_NOT_CONFIRMED (uint8_t)0xfe // was 0x03
#define SH_STATUS_FAILED        (uint8_t)0xff // was 0x01

#define NODEINFO_NODETYPE_CNTRL  (uint8_t)0x00
#define NODEINFO_NODETYPE_LIGHT  (uint8_t)0x01
#define NODEINFO_NODETYPE_FAN    (uint8_t)0x02
#define NODEINFO_NODETYPE_SERVER (uint8_t)0x03

#define SH_NODE_NUM_FOR_CNTRLR  1                  // For a wall switch "controller"
//#define SH_NODE_NUM_IDS  SH_NODE_NUM_FOR_CNTRLR  // For a wall switch "controller"
//#define SH_NODE_NUM_IDS  2                         // How many target loads on this node?

#define SH_NODE_TYPE_SERVER  0x00  // Linux Server (NOT an Arduino platform)
#define SH_NODE_TYPE_TARGET  0x01  // Target, has Triacs/PowerTail, NO LCD
#define SH_NODE_TYPE_CNTRL   0x02  // Wall "switch" controller, has LCD, uSD
#define SH_NODE_TYPE_MOBILE  0x03  // Android mobile device, accessing server node via internet


#define ROOM_FIRST (uint16_t)0
//#define DEFAULT_ROOM_NUM  (uint16_t)0
#define DEFAULT_ROOM_NUM  ROOM_FIRST
#define ROOM_CHANGE_ROTR (uint8_t)0x01
#define ROOM_CHANGE_ROTL (uint8_t)0x00

#define LOAD_FIRST (uint16_t)0
//#define DEFAULT_LOAD_NUM (uint16_t)0
#define DEFAULT_LOAD_NUM LOAD_FIRST
#define LOAD_CHANGE_ROTR (uint8_t)0x01
#define LOAD_CHANGE_ROTL (uint8_t)0x00
#define ROOM_NO_LOADS (uint16_t)0 //(uint16_t)0xffff //error code


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
//    volatile uint8_t   SHmsgType;              // current message status for this node
    SHmessage          SHthisNodeMsg;          // message fields for this node ID, to be in parallel to message fields of other Node IDs
    volatile uint8_t   newSHmsgTX;             // YES if a SH message is waiting to be sent, NO if not
    volatile uint8_t   newSHmsgRX;             // YES if a SH message has been received, NO if not
} SHnodeInfo, *prtSHnodeInfo;


// Is the load on or off? (regardless of current intensity value, so that we can ON back to the same intensity as when we went to OFF)
#define LOAD_POWERED_ON  HIGH //LOW //(uint8_t)0x01
#define LOAD_POWERED_OFF LOW  //HIGH //(uint8_t)0x00

// The various intensity (brightness/speed) levels
#define LOAD_INTENSITY_MIN       (uint16_t)0          // this level value is full-off
#define LOAD_INTENSITY_FULL_OFF  LOAD_INTENSITY_MIN   // this level value is full-off
#define LOAD_INTENSITY_LOW       (uint16_t)1          // this level value is full-off
#define LOAD_INTENSITY_MED_LOW   (uint16_t)2          // this level value is full-off
#define LOAD_INTENSITY_MED       (uint16_t)3          // this level value is full-off
#define LOAD_INTENSITY_MED_HIGH  (uint16_t)4          // this level value is full-off
#define LOAD_INTENSITY_MAX       (uint16_t)5          // this level value is full-on
#define LOAD_INTENSITY_FULL_ON   LOAD_INTENSITY_MAX   // this level value is full-on
#define LOAD_INTENSITY_HIGH      LOAD_INTENSITY_MAX   // this level value is full-on

#endif
