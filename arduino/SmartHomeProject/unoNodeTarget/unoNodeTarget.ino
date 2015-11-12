
/*
 *  NAME: unoNodeTarget
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
 *               Xbee module for Zigbee must be preconfigured as a Router in API mode and 9600 8n1
 *               
 *               This program was originally intended to use PWM outputs tocontrol triac, and this support up to 6 or so loads per Arduino Uno driver board.
 *               This proved problematic, and now the Triac control pulse is done in software, perhaps later as a different timer method than PWM, but
 *               pure software controlled bit-banging output pins, and this can allow a larger number of loads in Uno than OWM would have allowed.
 *               But for now, we remain using only one or two loads for testing/debug, and a large number may not be practical to make use of in real world
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


////#include "SmartHome_Zigbee.h"
#include <SmartHome_Zigbee.h>
SHzigbee mySHzigbee = SHzigbee();


// tried to move into a library but didn't get it right yet
// Arduino IDE may be flakey with fancy #ifs or #ifdef things
//#include <SmartHome_EEPROM.h>
#if 1

#include <SmartHome_NodeInfo.h>
#define SH_NODE_NUM_IDS  2                         // How many target loads on this node?
typedef struct
{
    volatile uint8_t numNodeIDs; // SH_NODE_NUM_IDS
    SHnodeInfo       nodeInfo[SH_NODE_NUM_IDS];
} SHnodeMasterInfo, *prtSHnodeMasterInfo;
SHnodeMasterInfo mySHnodeMasterInfo;

volatile uint8_t currentNodeInfoIndex= DEFAULT_LOAD_NUM;  // 


#ifdef uCtype uC_TYPE_UNO
// uncomment next line if want to init program a new Arduino Uno board for use
// Will ad dnumber of NodeIDs and fill in NodeInfo blocks for new and blank unit
#define INIT_UNO_EEPROM

#define UNO_SH_MAX_NODE_IDS               (uint16_t)SH_NODE_NUM_IDS  //(uint16_t)2

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

#define UNO_EEPROM_OFFSET_N0_BASE         (uint16_t)1  // base offset into SHnodeMasterInfo struct of the first load's NV EEPROM NodeInfo data

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

#endif // uCtype uC_TYPE_UNO
#endif // #if 0

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


// defines for Arduino digital pins from only the tumber to D and ht enumber, such as 6->D6
#define D6 6
#define D9 9


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
#define PIN_CTRL_LIGHT    (uint16_t)6 //D6
#define PIN_CTRL_FAN      (uint16_t)5 //D5  // was 9  //D9
//#define PIN_CTRL_FAN      (uint16_t)9  //D9
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



// standard pin13 LED on Arduino Uno and Due for testing and debug. NOT compatible with LCS panel installed on Due nodes.
uint16_t ledPin = 13;                 // LED connected to digital pin 13 for debug
uint8_t ledPinState = 0;

uint8_t i = 0; //for loop counter

// Initialize things at boot time, before starting the main program loop below
void setup() 
{
    uint8_t tmpNumNodes;

    inXbeeConfigMode   = NO; // default NOT in Xbee module config mode, so UART/serial port is communication between AVR and Xbee for normal usage

    // if need to program a blank Arduino Uno node for first use in this system
    #ifdef INIT_UNO_EEPROM
//        initEEPROMnodeInfo();
    #endif

    // put your setup code here, to run once:
    pinMode(ledPin, OUTPUT);      // sets the digital pin as output
    ledPinState = 0;

    // configure PWM things for load intensity control
    // NOTE: no longer using PWM, CLEANUP TODO
    //setupPWM();

    // configure manual pushbuttons (for debug controls and Xbee Config Mode selection)
    // as well as the AC ZeroCross event signal
    setupPCint();
    
    // Xbee should be preconfigured for API mode, 9600, 8n1, so match that in Arduino serial port
    Serial.begin(9600);

    Serial.println("Testing, 1, 2, 3, testing");
  
    initNodeInfoUno();

    //experimenting with sending an ON command message frame TOTO - remove experiment
    //zbXmitAPIframe();
}


// main program loop, iterate infinitely until/unless hit a hard exit
void loop() 
{
    uint8_t nodeIDnum = 0;

    // put your main code here, to run repeatedly:

    if(inXbeeConfigMode == NO)
    {
        // NOT in Xbee module config mode, so run the SmartHome program
        
        for (nodeIDnum = 0; nodeIDnum < mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)
        {
            // check current state, and maybe do something for this node ID if pending
            doNodeIDmsgSM(nodeIDnum);
        }

        // Check if already have received a new SmartHome message waiting to be processed
        if (mySHzigbee.newSHmsgRX == NO)
        {
//            Serial.print(";");
            //check for a new incoming frame data
            mySHzigbee.zbRcvAPIframe();
        }
        else // have received a Zigbee/SH message to process
        {
            for(i=0; i<mySHnodeMasterInfo.numNodeIDs; i++)
            {
                if(mySHzigbee.SHmsgRX.SHdestID == mySHnodeMasterInfo.nodeInfo[i].SHthisNodeID)
                {
                    captureRXmsg(i);
                }
            }
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
    pinMode(mySHnodeMasterInfo.nodeInfo[i].SHthisNodePin, OUTPUT);
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

//  tempNodeID = 0x0709;
  tempNodeID = 0xdead;
  programEEPROMnodeInfo( 0, 0, SH_NODE_TYPE_TARGET, PIN_CTRL_LIGHT, tempNodeID, LOAD_POWERED_OFF, 2, LOAD_INTENSITY_MAX );

//  tempNodeID = 0x0a0c;
  tempNodeID = 0xbeef;
  programEEPROMnodeInfo( 1, 0, SH_NODE_TYPE_TARGET, PIN_CTRL_FAN, tempNodeID, LOAD_POWERED_OFF, 8, LOAD_INTENSITY_MAX );

  Serial.println("Completed Initting EEPROM values");
}


// program the given EEPROM node info into EEPROM NV storage
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


// Do a SmartHome message conversation over Zigbee, if anything to do for the selected nodeInfo index
// This is Load-Driver unit specific, a Wall Control unit will have it's own implementation of something similar
void doNodeIDmsgSM(uint8_t nodeInfoIndex)
{
  volatile uint8_t   SHmsgNextState;         // which of 4 message stages are we in now, or 0=idle?
  volatile uint8_t   SHmsgCmd;
  volatile uint8_t   SHmsgStatus;

#if 0
    Serial.print("ENTERING doNodeIDmsgSM for nodeInfoIndex=");
    Serial.println(nodeInfoIndex, DEC);
#endif

#if 0
            Serial.print(" received SHmsg daddr/destID is ");
            Serial.print(mySHzigbee.SHmsgRX.SHdestID, HEX);
            Serial.print(" ?=? ID for node IDX ");
            Serial.print(nodeInfoIndex, DEC);
            Serial.print(" : ");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
            Serial.println();
#endif

    // update to previous iteration's next state
    //currentMsgState = nextMsgState;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgCurrentState = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState;

    switch ( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgCurrentState )
    {
        // Linux Server will receive ALL messages, and log ALL messages. No need to detect errors and specifically tell server of them.

        case SH_MSG_ST_IDLE:  // waiting for receive SH_MSG_ST_CMD_INIT message
#if 0
            Serial.print("ST_IDLE, newSHmsgRX=");
            Serial.print(mySHzigbee.newSHmsgRX, HEX);
            Serial.print(" ; SHmsgTypeRX=");
            Serial.print(mySHzigbee.SHmsgRX.SHmsgType, HEX);
            Serial.print(" ; SHdestIDrx=");
            Serial.print(mySHzigbee.SHmsgRX.SHdestID, HEX);
            Serial.print(" ?=? ");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
            Serial.println();
#endif

//            if ( (YES == mySHzigbee.newSHmsgRX) && (SH_MSG_TYPE_CMD_REQ == mySHzigbee.SHmsgRX.SHmsgType) && (mySHzigbee.SHmsgRX.SHdestID == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID) )
            if ( (YES == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgRX) && (SH_MSG_TYPE_CMD_REQ == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType) )
            {
                // TODO - going to keep it this way? - move to extractRXpayload? - keep this way for now
//                captureRXmsg(nodeInfoIndex);
                Serial.print("New CMD at nodeID=");
                Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
                Serial.print(" ?=? ");
                Serial.print(mySHzigbee.SHmsgRX.SHdestID, HEX);
                Serial.print(" ; from nodeID=");
                Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID, HEX);
                Serial.print(" ; CMD=");
                Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
                Serial.print(" ; chksum=");
                Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHchksum, HEX);

                mySHzigbee.newSHmsgRX = NO;
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgRX = NO;

                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_ACK_REQ;
            }
            else // this SH message was to a different node, do nothing this iteration
            {
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_IDLE;
            }
            break;

        case SH_MSG_ST_ACK_REQ:  // TX
            Serial.print("In SH_MSG_ST_ACK_REQ for nodeID ");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
            Serial.print(" for command ");
            Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
            
        //    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_CNFRM;

#if 0
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

            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_CNFRM;
            break;

        case SH_MSG_ST_CNFRM:  // RX
#if 0        
            if ( (mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgRX == YES) && (mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType == SH_MSG_TYPE_CONFIRM) && (mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHdestID == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID) )
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
#endif
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_COMPLETE;
            break;

        case SH_MSG_ST_COMPLETE: // TX

            // run the received SH command
            SHrunCommand(nodeInfoIndex);

            mySHzigbee.prepareTXmsg( 
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID,   // DestID is node that initiated this conversation
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID,             // Src ID is this node
                          SH_MSG_TYPE_COMPLETED,                                               // MsgType
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand,  // CMD
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH,
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL,
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal
                        );
            // indicate main loop that a TX frame is ready to send
//            newFrameForTX = YES;

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

            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_IDLE;
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand = SH_CMD_NOP;
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
#if 0
  prepareTXmsg( 
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID,   // DestID is node that initiated this conversation
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID,             // Src ID is this node
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
#endif

return(0);
}


// capture the received message into this Node ID's structure for processing inparallel to possible messages to another node
void captureRXmsg(uint8_t nodeInfoIndex)
{
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID      = mySHzigbee.SHmsgRX.SHsrcID; // SHsrcIDrx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType     = mySHzigbee.SHmsgRX.SHmsgType; // SHmsgTypeRX;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand     = mySHzigbee.SHmsgRX.SHcommand; // SHcommandRX;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH     = mySHzigbee.SHmsgRX.SHstatusH; // SHstatusHrx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL     = mySHzigbee.SHmsgRX.SHstatusL; // SHstatusLrx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusID    = ((mySHzigbee.SHmsgRX.SHstatusH << 8) | mySHzigbee.SHmsgRX.SHstatusL) ; // SHstatusIDrx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal   = mySHzigbee.SHmsgRX.SHstatusVal; // SHstatusValRX;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved1   = mySHzigbee.SHmsgRX.SHreserved1; // SHreserved1rx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved2   = mySHzigbee.SHmsgRX.SHreserved2; // SHreserved2rx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHchksum      = mySHzigbee.SHmsgRX.SHpayldChksum; // SHchksumRX;
//    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcalcChksum  = mySHzigbee.SHmsgRX.; // doCalcMsgChecksum(ZB_FRAME_TYPE_RX_RCVD);

    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgRX = YES;
}


// Determine which command is pending for this load ID and execute it
void SHrunCommand(uint8_t nodeInfoIndex)
{
  #if 1
    //switch ( SHcommandRX )
    switch ( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand )
    {
        case SH_CMD_LOAD_ON:
//            digitalWrite(ledPin, HIGH);   // sets the LED on
            loadPowerON(nodeInfoIndex);
            break;

        case SH_CMD_LOAD_OFF:
//      digitalWrite(ledPin, LOW);   // sets the LED off
        loadPowerOFF(nodeInfoIndex);
        break;

        case SH_CMD_LOAD_INC:
            loadIncreaseIntensity(nodeInfoIndex);
            break;
        
        case SH_CMD_LOAD_DEC:
            loadDecreaseIntensity(nodeInfoIndex);
            break;
        
        case SH_CMD_LOAD_GOTOFAV:    // change load to Favorite Intensity Level - NOT YET IMPLEMENTED
        case SH_CMD_LOAD_SAVEFAV:   // store new value as Favorite Intensity Level - NOT YET IMPLEMENTED
        case SH_CMD_LOAD_READFAV:   // send Favorite Intensity Level back to SH message source node - NOT YET IMPLEMENTED
        case SH_CMD_LOAD_READCRNT:  // send Current Intensity Level back to SH message source node - NOT YET IMPLEMENTED
        case SH_CMD_LOAD_EVNT_NOTICE:  // NEEDED? SERVER will log ALL messages, so may nto need to specifically send something to it
        case SH_CMD_NOP:  // No OPeration, DO NOTHING (same as default)
        default:
            // Unknown command, do nothing
            break;
    }

    Serial.println("");
    Serial.print("Ran SH cmd code 0x");
    Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
    Serial.println("");
#endif
}

#if 0
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
#endif


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


// Enable Pin-Change interrupts. These are NOT the int0 or int1 interrupt pins, 
// this allows the use of A0-A5 etc. as interrupt input pins as well.
void enablePCint(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


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

        // sanity check, reset to max intensith if somehow above that
        if(tmpVal > LOAD_INTENSITY_MAX)
        {
            tmpVal = LOAD_INTENSITY_MAX;
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = tmpVal;
//            EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), tmpVal );
        }
        
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

#if 0
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

// using PWM timers has been problematic, changing to a somewhat more manual pulse timing
    analogWrite(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodePin, tmpVal);
#endif

    // do not increase above max intensity
    if(tmpVal < LOAD_INTENSITY_MAX)
    {
        tmpVal += 1;
    }
    else
    {
        tmpVal = LOAD_INTENSITY_MAX;
    }

    // Update the current intensity level for this load
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = tmpVal;
//    EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), tmpVal );

    if(LOAD_INTENSITY_FULL_OFF < tmpVal)
    {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = YES;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), YES );        
    }
    else
    {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = NO;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), NO );              
    }

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

    tmpVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;

    // do not increase above max intensity
    if(tmpVal > LOAD_INTENSITY_MIN)
    {
        tmpVal -= 1;

        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = YES;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), YES );
    }
    else
    {
        tmpVal = LOAD_INTENSITY_MIN;
    }

    // Update the current intensity level for this load
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = tmpVal;
//    EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), tmpVal );

    if(LOAD_INTENSITY_FULL_OFF < tmpVal)
    {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = YES;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), YES );                
    }
    else
    {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = NO;
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), NO );                      
    }

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


#if 0
// trigger the AC line triacs to fire, turning on 
void enableSSRrelay(uint8_t nodeInfoIndex)
{
    Serial.println("DEBUG - In enableSSRrelay()");    
//    analogWrite(pin, 0); // disable the PWM to the load triacs
//    TODO - add PWM output pin ID to the load's NodeInfo structure in EEPROM, for analogWrite calls
// calculate new PWM value in places that change the current intensity, and in powerOn setup function
//    analogWrite(pin, newValue); // enable 
}
#endif


// Interrupt vector handling routine for A0 to A5 Pin Change Interrupt inputs
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
    #define DELAY_PUSHBTN_DELAY 100
    // The delay() calls below are flaky, since they use Timer0 and I've used the Timer0 PWM pins in conflict
    
    uint8_t AC0CrossCur = 0;

    AC0CrossCur = digitalRead(PIN_AC_ZERO_CROSS);
    
#if 1  
    // check if AC Zero cross trigger OR debug Blue button
    if( (AC0CROSS_AT_CROSSING == AC0CrossCur) && (AC0CROSS_NO_CROSSING == AC0CrossPrev) )
    {
        //enableSSRrelay(currentNodeInfoIndex);
        loadZeroCrossing();
//        Serial.print(".");
    }
    else
#else
        // check if debug Blue button
        if( (digitalRead(PIN_CHANGE_LOAD) == PIN_BUTTON_DOWN) && (buttonDimmerPrev == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(DELAY_PUSHBTN_DELAY);
            if(digitalRead(PIN_CHANGE_LOAD) == PIN_BUTTON_DOWN)
            {
                changeLoad(currentNodeInfoIndex);         
            }        
        }
#endif
    {

//        Serial.println("Entering pushbuttons ISR handler");

        // manual bushbuttons are used for early dev/debug of load control events processing.
        // Do software debounce on these pushbuttons. When move to other control methods later, 
        // comment out the pushbutton code or delete it. In final production system,
        // it is not expected to have these debounce delay statements here, which might affect the 
        // 60Hz AC Zero-Crossing trigger reliability.

#if 0  // Grey button toggles power to selected load
        // check if debug Grey button
        if( (digitalRead(PIN_ON_OFF) == PIN_BUTTON_DOWN) && (buttonOnOffPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(DELAY_PUSHBTN_DELAY);
            if(digitalRead(PIN_ON_OFF) == PIN_BUTTON_DOWN)
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadToggle(currentNodeInfoIndex);         
            }
        }
#else  // Grey button only turns power ON to load (not off for toggle)
        // check if debug Grey button
        if( (digitalRead(PIN_ON) == PIN_BUTTON_DOWN) && (buttonOnPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(DELAY_PUSHBTN_DELAY);
            if(digitalRead(PIN_ON) == PIN_BUTTON_DOWN)
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadPowerON(currentNodeInfoIndex);         
            }
        }
#endif

#if 0  // Yellow button toggles Xbee module config mode on Arduino UART
        // check if debug Yellow button
        if( (digitalRead(PIN_XBCONFIG) == PIN_BUTTON_DOWN) && (buttonXBconfigPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(DELAY_PUSHBTN_DELAY);
            if(digitalRead(PIN_XBCONFIG) == PIN_BUTTON_DOWN)
            {
                toggleXbeeConfigMode();          
            }
        }
#else  // Yellow button turns off the current load (selected by another manual pushbutton during debug)
        // check if debug Yellow button
        if( (digitalRead(PIN_OFF) == PIN_BUTTON_DOWN) && (buttonOffPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(DELAY_PUSHBTN_DELAY);
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
            delay(DELAY_PUSHBTN_DELAY);
            if(digitalRead(PIN_BRIGHER) == PIN_BUTTON_DOWN)
            {
                loadIncreaseIntensity(currentNodeInfoIndex);         
            }        
        }

        // check if debug Red button
        if( (digitalRead(PIN_DIMMER) == PIN_BUTTON_DOWN) && (buttonDimmerPrev   == PIN_BUTTON_UP) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
            delay(DELAY_PUSHBTN_DELAY);
            if(digitalRead(PIN_DIMMER) == PIN_BUTTON_DOWN)
            {
                loadDecreaseIntensity(currentNodeInfoIndex);         
            }        
        }

    }

    AC0CrossPrev = AC0CrossCur;

//    Serial.println("Exiting pushbuttons ISR handler");
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


// on a Zero Crossing detection from the 120V AC line waveform, 
// Set output drivign the trial to LOW immediately at Zero-Crossing time.
// Triac will stay on from a control signal of 1 until it hits another Zero-Crossing.
// So keep the control 0 until time to turn the triac on. The portion of the triac time on is duty cycle.
// Control should be a brief PULSE, so that it will be low again when the next Zero-Crossing occurs.
//    otherwise, if control is still a 1 when we get back here then the triac will be ON for the full time of this half-cycle.
// Triac control MUST be low at the Zer-Crossing.
// Once the Triac control goes high, the Triac will stay ON from then until next Zero-Crossing.
// Control to Triac should be a brief pulse high and then go low again, BEFORE the next Zero-Crossing.
// This was difficult/impossible to achieve using PWM outputs using Fast-PWM mode. Perhaps another mode would work better.
// This manual dummy-waiting loop seems to work, so continue with this.
void loadZeroCrossing(void)
{
    volatile uint32_t tmpCnt1 = 0;
    volatile uint32_t tmpCnt2 = 0;
    uint8_t nodeIDnum= 0;
    
    // for loop counters defining how long after Zero-Crossing we turn the Triac on for this half-cycle
    #define CNT_INTENSE_FULL_ON   0
    #define CNT_INTENSE_HIGH      1   // close to full-on
    // These counts stack onto each other, as well as the triac pulse width time, so 
    // that the Medium total count after Zero-Crossing is CNT_INTENSE_MED_HIGH + TRIAC_PULSE_WIDTH + CNT_INTENSE_MED
    #define CNT_INTENSE_MED_HIGH  500 //800
    #define CNT_INTENSE_MED       500 //600 //(1500 - CNT_INTENSE_MED_HIGH)
    #define CNT_INTENSE_MED_LOW   500 //100 //(2000 - CNT_INTENSE_MED)
    #define CNT_INTENSE_LOW       500 //(2500 - CNT_INTENSE_MED_LOW)
    #define TRIAC_PULSE_WIDTH     100

    
    // full-ON loads (should never be NOT enabled)
    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_FULL_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
            {
                // these should not get turned off, if full-ON
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON); 
            }
            else  // LOAD_POWERED_OFF
            {
                // these should stay OFF
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);               
            }
        }
    }


    // ----------------
    // medium-high intensity loads - most intense while not full-ON

    for(tmpCnt1=0; tmpCnt1<CNT_INTENSE_MED_HIGH; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_MED_HIGH == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
            {
                // Triac pulse ON
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON); 
            }
            else  // LOAD_POWERED_OFF
            {
                // these should stay OFF
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);                             
            }
        }
    }

    // pulse high to Triac control
    for(tmpCnt1=0; tmpCnt1<TRIAC_PULSE_WIDTH; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_MED_HIGH == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            // Triac pulse OFF
            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF); 
        }
    }


    // ----------------
    // medium intensity loads

    for(tmpCnt1=0; tmpCnt1<CNT_INTENSE_MED; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_MED == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
            {
                // Triac pulse ON
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON); 
            }
            else  // LOAD_POWERED_OFF
            {
                // these should stay OFF
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);                             
            }
        }
    }

    // pulse high to Triac control
    for(tmpCnt1=0; tmpCnt1<TRIAC_PULSE_WIDTH; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_MED == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            // Triac pulse OFF
            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF); 
        }
    }

    
    // ----------------
    // medium-low intensity loads

    for(tmpCnt1=0; tmpCnt1<CNT_INTENSE_MED_LOW; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_MED_LOW == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
            {
                // Triac pulse ON
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON); 
            }
            else  // LOAD_POWERED_OFF
            {
                // these should stay OFF
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);                             
            }
        }
    }

    // pulse high to Triac control
    for(tmpCnt1=0; tmpCnt1<TRIAC_PULSE_WIDTH; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_MED_LOW == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            // Triac pulse OFF
            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF); 
        }
    }


    // ----------------
    // low intensity loads

    for(tmpCnt1=0; tmpCnt1<CNT_INTENSE_LOW; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_LOW == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
            {
                // Triac pulse ON
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON); 
            }
            else  // LOAD_POWERED_OFF
            {
                // these should stay OFF
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);                             
            }
        }
    }

    // pulse high to Triac control
    for(tmpCnt1=0; tmpCnt1<TRIAC_PULSE_WIDTH; tmpCnt1++); 

    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( LOAD_INTENSITY_LOW == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
        {
            // Triac pulse OFF
            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF); 
        }
    }


    // ----------------
    // full-OFF intensity loads

    //for(tmpCnt1=0; tmpCnt1<CNT_INTENSE_FULL_OFF; tmpCnt1++); 

    // These loads are NEVER on at all
    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)    
    {
        if( (LOAD_INTENSITY_FULL_OFF == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent) ||
            (LOAD_POWERED_OFF == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered) )
        {
            // Triac pulse OFF
            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF); 
        }
    }
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
//    digitalWrite(PIN_AC_ZERO_CROSS,LOW);  // Blue
//    digitalWrite(PIN_CHANGE_LOAD,HIGH);  // Blue
//    digitalWrite(PIN_XBCONFIG,HIGH);       // Yellow
    digitalWrite(PIN_OFF,HIGH);            // Yellow

    
    // enable Pin Change Interrupt for pin...
//    enablePCint(PIN_ON_OFF);
    enablePCint(PIN_ON);
    enablePCint(PIN_BRIGHER);
    enablePCint(PIN_DIMMER);
    enablePCint(PIN_AC_ZERO_CROSS);
//    enablePCint(PIN_CHANGE_LOAD);
//    enablePCint(PIN_XBCONFIG);
    enablePCint(PIN_OFF);
}


