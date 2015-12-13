
/*
 *  NAME: unoNodeTarget
 *  DESCRIPTION: Arduino code for an Uno R3 type Arduino board, in the SmartHome lighting and ceiling fan control system for
 *               Bill Toner's Fall 2015 Embedded Systems project EN.525.743 at Johns Hopkins University, Dorsey Center
 *               with Professor Houser.
 *
 *               While Arduino preference is to do things in a very C++ style, I am a C programmer, and will
 *               write some of this software in more of a C style, due to scheduling concerns of working in
 *               a style that is slightly foreign to me.
 *
 *               Arduino Uno Target nodes in this system will be directly in control of the brightness of light fixtures,
 *               the speed of ceiling fans, and communicate with other portions of the SmartHome system via
 *               Zigbee wireless messages, using the Zigbee API mode TX Request and RX Received frames.
 *               Other Zigbee API frame types will be ignored. All Zigbee messages will be sent to broadcast address
 *               so that all other nodes will receive the same message.
 *
 *               Target loads, light fixtures or ceiling fans, will be driven by the Uno board via
 *               a set of ZeroCross Tail and PowerSSR Tail (triac relay) units to observe and control the
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
 *               This unit's Xbee Zigbee module MUST and SHALL BE preconfigured as a Router in API mode and 9600 8n1
 *               One XBee Zigbee module, attached to the SmartHome Linux Server unit, MUST and SHALL be preconfigured as a Coordinator Router in API mode and 9600 8n1
 *               
 *               This program was originally intended to use PWM outputs tocontrol triac, and this support up to 6 or so loads per Arduino Uno driver board.
 *               This proved problematic, and now the Triac control pulse is done in software, perhaps later as a different timer method than PWM, but
 *               pure software controlled bit-banging output pins, and this can allow a larger number of loads in Uno than OWM would have allowed.
 *               But for now, we remain using only one or two loads for testing/debug, and a large number may not be practical to make use of in real world
 *               
 *               This program makes use of the TimerOne library, which is made available under the
 *               Creative Commons Attribution 3.0 United States License  "CC BY 3.0 US"
 *               https://github.com/PaulStoffregen/TimerOne
 *               http://creativecommons.org/licenses/by/3.0/us/
 *
 */

// uncomment these DEBUG items to enable debug serial prints
//#define DEBUG_ZB_RX
//#define DEBUG_ZB_TX


// Select the Microcontroller type for this node unit
#define uC_TYPE_UNO   1  // 8bit AVR used for load Targets
#define uC_TYPE_MEGA  2  // 8bit AVR with larger memory and more board pins than Uno
#define uC_TYPE_DUE   3  // 32bit ARM CortexM3 used for Wall "switch" controllers
#define uC_TYPE_X64   4  // 64bit x86/x64 PC Linux Server

#define uCtype      uC_TYPE_UNO
//#define uCtype      uC_TYPE_MEGA
//#define uCtype      uC_TYPE_DUE
//#define uCtype      uC_TYPE_X64


// Include Arduino standard libraries
#include "Arduino.h"

// Because Unos and Dues have different sizes for int
#include "stdint.h"

// Include 3rd party libraries (downloaded, not created by me and not part of Arduino standard)
#include <TimerOne.h>
// Timer1 instance is defined in the TimerOne.h file above, use that


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


// defines for Arduino digital pins from only the tumber to D and the number, such as 6->D6
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

// Trigger signal from ZeroCross Tail unit for AC line sinewave timing of Triac AC relay control signals
volatile uint8_t AC0CrossPrev         = AC0CROSS_NO_CROSSING;

// Timer1 variables for timed triac controls
#define PERIOD_uS_60HZ  (unsigned long)(1000000 * 1 / 60)  // comes out in microseconds
#define HALFPERIOD_uS_60HZ (unsigned long)(PERIOD_uS_60HZ / 2)  // comes out in microseconds
//#define T1_LOAD_INTENSITY_TIMESTEP (unsigned long)( (HALFPERIOD_uS_60HZ / 5) - 120 )  // comes out in microseconds
//#define T1_LOAD_INTENSITY_TIMESTEP (unsigned long)( 1500 )  // comes out in microseconds
#define T1_LOAD_INTENSITY_TIMESTEP (long)( 30000 )  // comes out in microseconds
//#define T1_LOAD_INTENSITY_TIMESTEP (unsigned long)( 8000 )  // in microseconds
#define TRIAC_FIRE_PULSE_TIME (unsigned long)(100)
volatile uint8_t shTriacPulse = YES;
volatile uint8_t shIrqT1thisLevel = LOAD_INTENSITY_FULL_OFF;

// standard pin13 LED on Arduino Uno and Due for testing and debug. NOT compatible with LCS panel installed on Due nodes.
uint16_t ledPin = 13;                 // LED connected to digital pin 13 for debug
volatile uint8_t ledPinState = 0;
uint16_t pinTriacIRQ  = 4;         // for debug to indicate when in the Triac timer IRQ
uint16_t pinZeroCrossIRQ  = 7;         // for debug to indicate when in the Triac timer IRQ
uint16_t pinTimer1irq = 8;         // for debug to indicate when in the Triac timer IRQ


uint8_t i = 0; //for loop counter

// Initialize things at boot time, before starting the main program loop below
void setup()
{
    uint8_t tmpNumNodes;

    inXbeeConfigMode   = NO; // default NOT in Xbee module config mode, so UART/serial port is communication between AVR and Xbee for normal usage

    // if need to program a blank Arduino Uno node for first use in this system
    // disable/comment out the initEEPROMnodeInfo() call if EEPROM has already been programmed with SmartHome/load data for this unit
    #ifdef INIT_UNO_EEPROM
        initEEPROMnodeInfo();
    #endif

    // put your setup code here, to run once:

    // some debug output signals for probing with LED or oscilloscope
    pinMode(pinZeroCrossIRQ, OUTPUT);      // sets the digital pin as output
    digitalWrite(pinZeroCrossIRQ, LOW);   // sets the pin "off"
    pinMode(pinTimer1irq, OUTPUT);      // sets the digital pin as output
    digitalWrite(pinTimer1irq, LOW);   // sets the pin "off"
    pinMode(pinTriacIRQ, OUTPUT);      // sets the digital pin as output
    digitalWrite(pinTriacIRQ, LOW);   // sets the pin "off"
    pinMode(ledPin, OUTPUT);      // sets the digital pin as output
    ledPinState = 0;

    // configure manual pushbuttons (for debug controls and Xbee Config Mode selection)
    // as well as the AC ZeroCross event signal
    setupPCint();

    // Xbee should be preconfigured for API mode, 9600, 8n1, so match that in Arduino serial port
    Serial.begin(9600);
//    Serial.begin(57600);

    Serial.println("Testing, 1, 2, 3, testing");

    //init the nodeInfo structure data 
    initNodeInfoUno();
    
Serial.print("PERIOD_uS_60HZ = ");
Serial.println(PERIOD_uS_60HZ, DEC);

Serial.print("HALFPERIOD_uS_60HZ = ");
Serial.println(HALFPERIOD_uS_60HZ, DEC);

Serial.print("Triac timestep = ");
Serial.println(T1_LOAD_INTENSITY_TIMESTEP, DEC);


    noInterrupts();
    Timer1.initialize(); // done once to init the timer library, later use setperiod to change
    Timer1.attachInterrupt(irqT1triacTriggers); // irqT1triacTriggers to be called at each intensity level timestep AND at end of each triac fire pulse
    Timer1.setPeriod(1200);
    Timer1.start();
    interrupts();

Serial.print("F_CPU=");
Serial.print(F_CPU, DEC);
Serial.print(" ; TIMER1_RESOLUTION=");
Serial.print(TIMER1_RESOLUTION, DEC);
Serial.print(" ; TIMSK1=b");
Serial.print(TIMSK1, BIN);
Serial.print(" ; ICR1=");
Serial.print(ICR1 , DEC);
Serial.print(" ; TCCR1A=b");
Serial.print(TCCR1A , BIN);
Serial.print(" ; TCCR1B=b");
Serial.print(TCCR1B , BIN);
Serial.print(" ; WGM13=");
Serial.print(WGM13, DEC);
Serial.println();

    shTriacPulse= NO;
    shIrqT1thisLevel = LOAD_INTENSITY_FULL_OFF;

    //experimenting with sending an ON command message frame TOTO - remove experiment
    //zbXmitAPIframe();
}


// main program loop, iterate infinitely until/unless hit a hard exit
void loop()
{
    uint8_t nodeIDnum = 0;

    // put your main code here, to run repeatedly:

//    delay(80);  // experiment to see if ZB receive can work at all and investigate timing issues

#if 1
    if(NO == inXbeeConfigMode)
    {
        // NOT in Xbee module config mode, so run the SmartHome program


        // Check if already have received a new SmartHome message waiting to be processed
        // Keep teh ZB Receive call first in line here, as it is most sensitive to timing
        //and dropped bytes if we take too long between reads
        // it would be nice to be able to read from serial based on interrupts, either from RXbuff content
        // or from a timer to get us back here frequently enough if there are long/slow chunks of code somewhere
        // to get lost in. I don't know how to do that in Arduino though.
        if (NO == mySHzigbee.newSHmsgRX)
        {
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

#if 0
        // check if have a SmartHome message waiting to be sent
        if (YES == mySHzigbee.newSHmsgTX)
        {
            // ?assemble? and transmit the frame
            mySHzigbee.zbXmitAPIframe();
        }
#endif

        for (nodeIDnum = 0; nodeIDnum < mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)
        {
            // check current state, and maybe do something for this node ID if pending
            doNodeIDmsgSM(nodeIDnum);
        }
    }
#endif

#if 0
    if(Serial.available() > 0)
    {
        while(Serial.available() > 0)
        {
            Serial.print(Serial.read(), HEX);
            Serial.print(" ");
        }
        Serial.println("<>");
    }
#endif

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

#if 0
    Serial.print("DEBUG - setting FAV level for node ");
    Serial.print(i, DEC);
    Serial.print(" to level ");
    Serial.print(mySHnodeMasterInfo.nodeInfo[i].SHthisNodeLevelFav);
    Serial.println();
#endif

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
  programEEPROMnodeInfo( 0, 0, SH_NODE_TYPE_TARGET, PIN_CTRL_LIGHT, tempNodeID, LOAD_POWERED_OFF, 2, 4 );

//  tempNodeID = 0x0a0c;
  tempNodeID = 0xbeef;
  programEEPROMnodeInfo( 1, 0, SH_NODE_TYPE_TARGET, PIN_CTRL_FAN, tempNodeID, LOAD_POWERED_OFF, 5, 2 );

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

        case SH_MSG_ST_ACK_REQ:  // TX back to initiator
            Serial.print("In SH_MSG_ST_ACK_REQ for nodeID ");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
            Serial.print(" for command ");
            Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);

#if 0
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType = SH_MSG_ST_ACK_REQ;
            mySHzigbee.prepareTXmsg(
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID,   // DestID is node that initiated this conversation
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID,             // Src ID is this node
                          SH_MSG_ST_ACK_REQ,                                                   // MsgType
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand,  // CMD
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH,
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL,
                          mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal
                        );
            // indicate main loop that a TX frame is ready to send
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgTX = YES;
            mySHzigbee.zbXmitAPIframe();      // Transmit the SmartHome message over Zigbee

            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgTX = NO;  // sent it, so make sure we don't send it again

            // wait fro confirmation from command initiator node
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_CNFRM;

#else
            // quick hack for development/debug while we get this protocol working
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_CNFRM;
#endif

            break;

        case SH_MSG_ST_CNFRM:  // RX
#if 0        
            if ( (YES == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgRX) && (SH_MSG_TYPE_CONFIRM == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType) ) // && 
            {
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_COMPLETE;
            }
#else        
            // quick hack for development/debug while we get this protocol working
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHmsgNextState = SH_MSG_ST_COMPLETE;
#endif
            break;

        case SH_MSG_ST_COMPLETE: // TX
#if 0 //to be enabled when other levels of protocol are implemented       
            if(SH_STATUS_CONFIRMED == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal)
            {
                // run the received SH command
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = SHrunCommand(nodeInfoIndex);
            }
            else
            {
                mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = SH_STATUS_FAILED;
            }
#else
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = SHrunCommand(nodeInfoIndex);
#endif
            
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
//            mySHzigbee.newSHmsgTX = YES;
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgTX = YES;
            mySHzigbee.zbXmitAPIframe();      // Transmit the SmartHome message over Zigbee
            
            Serial.print("nodeID=");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeID, HEX);
            Serial.print(" sending COMPLETED to ");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHothrID, HEX);
            Serial.print(" ; msgType=");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHmsgType, HEX);
            Serial.print(" ; CMD=");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
            Serial.print(" ; statusH=");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH, HEX);
            Serial.print(" ; statusL=");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL, HEX);
            Serial.print(" ; status=");
            Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal, HEX);
            Serial.println();

            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgTX = NO;  // sent it, so make sure we don't send it again

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
// deprecated    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusID    = ((mySHzigbee.SHmsgRX.SHstatusH << 8) | mySHzigbee.SHmsgRX.SHstatusL) ; // SHstatusIDrx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal   = mySHzigbee.SHmsgRX.SHstatusVal; // SHstatusValRX;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved1   = mySHzigbee.SHmsgRX.SHreserved1; // SHreserved1rx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHreserved2   = mySHzigbee.SHmsgRX.SHreserved2; // SHreserved2rx;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHchksum      = mySHzigbee.SHmsgRX.SHpayldChksum; // SHchksumRX;

    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].newSHmsgRX = mySHzigbee.newSHmsgRX; //YES; ??
}


// Determine which command is pending for this load ID and execute it
uint8_t SHrunCommand(uint8_t nodeInfoIndex)
{
    uint8_t cmdStatus = SH_STATUS_FAILED; // init to Failed, change it if something goes well
  #if 1
    //switch ( SHcommandRX )
    switch ( mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand )
    {
        case SH_CMD_LOAD_ON:
//            digitalWrite(ledPin, HIGH);   // sets the LED on
            cmdStatus = loadPowerON(nodeInfoIndex);
            break;

        case SH_CMD_LOAD_OFF:
//          digitalWrite(ledPin, LOW);   // sets the LED off
            cmdStatus = loadPowerOFF(nodeInfoIndex);
            break;

        case SH_CMD_LOAD_INC:
            cmdStatus = loadIncreaseIntensity(nodeInfoIndex);
            break;

        case SH_CMD_LOAD_DEC:
            cmdStatus = loadDecreaseIntensity(nodeInfoIndex);
            break;

        case SH_CMD_LOAD_READFAV:   // send Favorite Intensity Level back to SH message source node - NOT YET IMPLEMENTED
        case SH_CMD_LOAD_READPWR:   // send isPowered state back to SH message source node - NOT YET IMPLEMENTED
        case SH_CMD_LOAD_READCRNT:  // send Current Intensity Level back to SH message source node - NOT YET IMPLEMENTED
            cmdStatus = returnPoweredAndCurrentAndFavLevels(nodeInfoIndex);
            break;

        case SH_CMD_LOAD_GOTOFAV:    // change load to Favorite Intensity Level - NOT YET IMPLEMENTED
            cmdStatus = loadGotoFavIntensity(nodeInfoIndex);
            break;

        case SH_CMD_LOAD_SAVEFAV:   // store new value as Favorite Intensity Level - NOT YET IMPLEMENTED
            cmdStatus = saveCurrentAsNewFavLevel(nodeInfoIndex);
            break;
            
// deprecated        case SH_CMD_LOAD_EVNT_NOTICE:  // NEEDED? SERVER will log ALL messages, so may nto need to specifically send something to it

        case SH_CMD_NOP:  // No OPeration, DO NOTHING (same as default)
        default:
            // Unknown command, do nothing
            break;
    }

    // update the message fields for transmit
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH   = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL   = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;


    Serial.println("");
    Serial.print("Ran SH cmd code 0x");
    Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHcommand, HEX);
    Serial.println("");
#endif

    return(cmdStatus);
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
  if (0 == ledPinState)
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
    if(LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered)  // Load is currently powered ON (possibly dim/slow, but ON)
    {
        // Turn it off
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = LOAD_POWERED_OFF;
//        EEPROM.update(UNO_EEPROM_OFFSET_BS_POWERED, PIN_LED_OFF);
        digitalWrite(PIN_CTRL_LIGHT, PIN_LED_OFF);
    }
    else  // Load is currently powered full-OFF
    {
        // Turn it ON (not necessarily full-ON, may be dimmed/slow intensity level, but not full-OFF
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = LOAD_POWERED_ON;
//        EEPROM.update(UNO_EEPROM_OFFSET_BS_POWERED, PIN_LED_ON);
        digitalWrite(PIN_CTRL_LIGHT, PIN_LED_ON);
    }

    Serial.print("DEBUG - In loadToggle(), new intensity value=");
    Serial.print(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered, DEC);
    Serial.print("for NodeInfoIndex=");
    Serial.println(nodeInfoIndex, DEC);

//    EEPROM.read(); // get new value just saved from EEPROM

// LOAD_POWERED_ON

// TODO - EEPROM needs another byte per load to save on/off state as well as current intensity. If turn off, store that, but also keep current intensity for next on.
}



// change the current active intensity level to the stored favorite level for this node
uint8_t loadGotoFavIntensity(uint8_t nodeInfoIndex)
{
    uint16_t tmpVal = 0;

    tmpVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;

    if(tmpVal != mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav)
    {
        // update current level from whatever to FAVorite level
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav;

        // Save the new current intensity level to EEPROM for erecovery from a power outage
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), tmpVal );
    }

    // if not already on, turn it on to the new level
    loadPowerON(nodeInfoIndex);

    return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent);
}


// change the current favorite intensity level to the current active level for this node
uint8_t loadSaveNewFavIntensity(uint8_t nodeInfoIndex)
{
    uint16_t tmpVal = 0;

    tmpVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav;

    if(tmpVal != mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent)
    {
        // update current level from whatever to FAVorite level
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;

        // Save the new current intensity level to EEPROM for erecovery from a power outage
//        EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_FAV_INTSTY), tmpVal );
    }

    // if not already on, turn it on to the new level
    loadPowerON(nodeInfoIndex);

    return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav);
}


// If load is currently full-OFF, then apply power to previous current intensity level
// (ON/OFF is essentially an enable condition to the current intensity level)
uint8_t loadPowerON(uint8_t nodeInfoIndex)
{
    uint16_t tmpVal = 0;

    Serial.print("DEBUG - In loadPowerON()");

    if(NO == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered)
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

    return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent);
}


// If load is currently full-OFF, then apply power to previous current intensity level
// (ON/OFF is essentially an enable condition to the current intensity level)
uint8_t loadPowerOFF(uint8_t nodeInfoIndex)
{
    Serial.print("DEBUG - In loadPowerOFF()");

    if(YES == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered)
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

    return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent);
}


// increase intensity level one step
uint8_t loadIncreaseIntensity(uint8_t nodeInfoIndex)
{
//    uint16_t tmpVal = 0;
    uint8_t tmpVal = 0;

//mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodePin          = EEPROM.read(eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_PIN);

    Serial.print("DEBUG - In loadIncreaseIntensity()");

    if(NO == mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered)
    {
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = LOAD_INTENSITY_FULL_OFF + 1;
        mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = YES;
    }
    else
    {
        if(LOAD_INTENSITY_MAX <= mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent)
        {
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent = LOAD_INTENSITY_MAX;
        }
        else
        {
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent += 1;
        }


        if(LOAD_INTENSITY_FULL_OFF < mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent)
        {
            mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered = YES;
        }
    }

//    EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_BS_CRNT_INTSTY), mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent );
//    EEPROM.update( (eepromOffsetNodeBase + UNO_EEPROM_OFFSET_N1_POWERED), YES );

    Serial.print(" ; Load ");
    Serial.print(nodeInfoIndex, DEC);
    Serial.print(" new intensity = ");
    Serial.println(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent, DEC);

    return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent);
}


// Save the current intensity level as the new FAVorite level
uint8_t saveCurrentAsNewFavLevel(uint8_t nodeInfoIndex)
{
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;

    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH   = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL   = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;
    
    return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav);
}


// return the current level, isPowered state, and FAVorite level values to the inquiring node
uint8_t returnPoweredAndCurrentAndFavLevels(uint8_t nodeInfoIndex)
{
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusH   = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelFav;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusL   = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeIsPowered;
    mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeMsg.SHstatusVal = mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent;
    
    return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent);
}


// decrease intensity level one step
uint8_t loadDecreaseIntensity(uint8_t nodeInfoIndex)
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

    if(LOAD_INTENSITY_FULL_OFF < tmpVal)
    {
        return(mySHnodeMasterInfo.nodeInfo[nodeInfoIndex].SHthisNodeLevelCurrent);
    }
    else
    {
        return(LOAD_INTENSITY_FULL_OFF);
    }
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
        if( (PIN_BUTTON_DOWN == digitalRead(PIN_CHANGE_LOAD)) && (PIN_BUTTON_UP == buttonDimmerPrev) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
//            delay(DELAY_PUSHBTN_DELAY);
            if(PIN_BUTTON_DOWN == digitalRead(PIN_CHANGE_LOAD))
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
        if( (PIN_BUTTON_DOWN == digitalRead(PIN_ON_OFF)) && (PIN_BUTTON_UP == buttonOnOffPrev) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
 //           delay(DELAY_PUSHBTN_DELAY);
            if(PIN_BUTTON_DOWN == digitalRead(PIN_ON_OFF))
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadToggle(currentNodeInfoIndex);
            }
        }
#else  // Grey button only turns power ON to load (not off for toggle)
        // check if debug Grey button
        if( (PIN_BUTTON_DOWN == digitalRead(PIN_ON)) && (PIN_BUTTON_UP == buttonOnPrev) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
//            delay(DELAY_PUSHBTN_DELAY);
            if(PIN_BUTTON_DOWN == digitalRead(PIN_ON))
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadPowerON(currentNodeInfoIndex);
            }
        }
#endif

#if 0  // Yellow button toggles Xbee module config mode on Arduino UART
        // check if debug Yellow button
        if( (PIN_BUTTON_DOWN == digitalRead(PIN_XBCONFIG)) && (PIN_BUTTON_UP == buttonXBconfigPrev) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
//            delay(DELAY_PUSHBTN_DELAY);
            if(PIN_BUTTON_DOWN == digitalRead(PIN_XBCONFIG))
            {
                toggleXbeeConfigMode();
            }
        }
#else  // Yellow button turns off the current load (selected by another manual pushbutton during debug)
        // check if debug Yellow button
        if( (PIN_BUTTON_DOWN == digitalRead(PIN_OFF)) && (PIN_BUTTON_UP == buttonOffPrev) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
//            delay(DELAY_PUSHBTN_DELAY);
            if(PIN_BUTTON_DOWN == digitalRead(PIN_OFF))
            {
                // (ON/OFF is essentially an enable condition to the current intensity level)
                loadPowerOFF(currentNodeInfoIndex);
            }
        }
#endif


        // check if debug Green button
        if( (PIN_BUTTON_DOWN == digitalRead(PIN_BRIGHER)) && (PIN_BUTTON_UP == buttonBrighterPrev) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
//            delay(DELAY_PUSHBTN_DELAY);
            if(PIN_BUTTON_DOWN == digitalRead(PIN_BRIGHER))
            {
                loadIncreaseIntensity(currentNodeInfoIndex);
            }
        }

        // check if debug Red button
        if( (PIN_BUTTON_DOWN == digitalRead(PIN_DIMMER)) && (PIN_BUTTON_UP == buttonDimmerPrev) )
        {
            // delay and check again for software debouncing (do not debounce ACzeroCross from ZeroCross Tail)
//            delay(DELAY_PUSHBTN_DELAY);
            if(PIN_BUTTON_DOWN == digitalRead(PIN_DIMMER))
            {
                loadDecreaseIntensity(currentNodeInfoIndex);
            }
        }

    }

    AC0CrossPrev = AC0CrossCur;

//    Serial.println("Exiting pushbuttons irq ISR handler");
}


// toggle between Xbee config mode and normal running mode.
// when Xbee Config mode is active, the AVR microcontroller should go "dormant", the SmartHome software will halt/pause execution,
// and stay off the uart serial port, so that the serial port is used to configure the Xbee module on the board using XCTU from a PC.
// When Xbee config mode is NOT active, then the SmartHome software will run normally and make use of the Xbee module for communications.
void toggleXbeeConfigMode(void)
{
    if(NO == inXbeeConfigMode)  // Xbee config mode is currently DISabled
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
// if triac control is HIGH then triac will be ON for full 60Hz half-period until next Zero-Cross event state
// For any load needing possibility to be off for part or all of the half-period, then the triac control needs to be LOW *-before-* the Zero-Cross event
// Triac will stay on from a control signal of HIGH until it hits another Zero-Crossing.
// So keep the control 0 until time to turn the triac on. The portion of the triac time on is duty cycle.
// Control should be a brief PULSE, so that it will be low again when the next Zero-Crossing occurs.
//    otherwise, if control is still a 1 when we get back here then the triac will be ON for the full time of this half-cycle.
// Triac control MUST be low at the Zero-Crossing unless the load should be always-ON.
// Once the Triac control goes high, the Triac will stay ON from then until next Zero-Crossing.
// Control to Triac should be a brief pulse high and then go low again, BEFORE the next Zero-Crossing.
// This was difficult/impossible to achieve using PWM outputs using Fast-PWM mode. Perhaps another mode would work better.
// The manual dummy-waiting loop seems to work for itself,  but interferes with receiving serial data due to time spent in this irq handler that way
// use Timer1 irqs instead of software waiting loops here to help with serial port timing
void loadZeroCrossing(void)
{

#define TRIAC_FROM_TIMER1
#ifdef TRIAC_FROM_TIMER1
    uint8_t nodeIDnum= 0;

    // stop Timer1 so it doesn't accidentally trigger irq while we are in here
//    Timer1.stop();

    // set Timer1 to signal irq handler at timestep for next intensity level
    // This should reset the Timer1 wherever it is and give a consistent
    // timestep starting from here, at the beginning of an AC power half-cycle
//    Timer1.setPeriod(T1_LOAD_INTENSITY_TIMESTEP);
//    Timer1.restart();
    Timer1.start();

    digitalWrite(pinZeroCrossIRQ, HIGH);   // sets the debug pin "on"

    // on a Zero-cross trigger, start looking at FULL-ON intensity level, and decrement from there
    shIrqT1thisLevel = LOAD_INTENSITY_FULL_ON;

    // full-ON loads (should be ALWAYS on at this intensity level unless the IsPowered indicator says turned off)
    for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)
    {
        if( LOAD_INTENSITY_FULL_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent ) // sanity check
        {
            if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
            {
                // these should not get turned off by triac, if full-ON
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON);
            }
            else  // LOAD_POWERED_OFF
            {
                // these should stay OFF by triac
                digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);
            }
        }
        else if( (LOAD_INTENSITY_FULL_OFF >= mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent) || 
                 (LOAD_POWERED_OFF == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered) )
        {
            // if the load is OFF then turn off its AC relay PowerSSR Tail unit
            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);        
        }
    }

    // Timer1 irqs should take it from here until next zero-cross event

#else
// old method using software wait loops for timing. Works alone, but uses up too much time for serial data to be received, some bytes are lost there this way
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
#endif

    digitalWrite(pinZeroCrossIRQ, LOW);   // sets the debug pin "off"
    
} // loadZeroCrossing()


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


// IRQ handler for Timer1 events
void irqT1triacTriggers(void)
{
    noInterrupts();
    
    uint8_t nodeIDnum= 0;  

    // stop Timer1 so does not accidentally trigger irq while we are in here
    Timer1.stop();

    if(LOAD_INTENSITY_FULL_OFF < shIrqT1thisLevel)
    {
        // set Timer1 to signal irq handler at timestep for next intensity level
//        Timer1.setPeriod(T1_LOAD_INTENSITY_TIMESTEP);
//        Timer1.restart();
        Timer1.start();
    }

    digitalWrite(pinTimer1irq, HIGH);   // sets the pin "on"

#if 1
//    Serial.print("[");
//    Serial.print(shIrqT1thisLevel, DEC);
////    Serial.print("-");
//    Serial.print(shTriacPulse, DEC);
//    Serial.print("]");
//    Serial.print(".");
#endif
    
    switch(shIrqT1thisLevel)
    {
        case LOAD_INTENSITY_FULL_ON: // don't expect to end up in here for FULL-ON loads, but do this as sanity check
            // full-ON loads (should be ALWAYS on at this intensity level unless the IsPowered indicator says turned off)
            for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)
            {
                if( LOAD_INTENSITY_FULL_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
                {
                    if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
                    {
                       // these should not get turned off by triac, if full-ON
                        digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON);
                    }
                    else  // LOAD_POWERED_OFF
                    {
                        // these should stay OFF by triac
                        digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);
                    }
                }
            }

            shIrqT1thisLevel--;
            
            break;

        case LOAD_INTENSITY_MED_HIGH:  // medium-high intensity loads - most intense while not full-ON
        case LOAD_INTENSITY_MED:       // medium intensity loads
        case LOAD_INTENSITY_MED_LOW:   // medium-low intensity loads
        case LOAD_INTENSITY_LOW:       // low intensity loads

                // beginning a triac fire pulse for this intensity level

                // turn ON triac control for any loads getting a triac pulse for this intensity level
                for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)
                {
                    if( shIrqT1thisLevel == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
                    {
                        if( LOAD_POWERED_ON == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeIsPowered )
                        {
                            // these should not get turned ON by triac at this time
                            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_ON);
                        }
                        else  // LOAD_POWERED_OFF
                        {
                            // these should be OFF by triac
                            digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);
                        }
                    }
                }

                shIrqT1thisLevel--;                

                // leave the trigger pulse on until we get to the FULL-OFF/MIN level check below, and turn them ALL off at that point
                // while that point SHALL come BEFORE the next Zero-Cross trigger
            break;

        case LOAD_INTENSITY_FULL_OFF:  // full-OFF intensity loads

//            Timer1.setPeriod(T1_LOAD_INTENSITY_TIMESTEP);
//            Timer1.restart();

            for (nodeIDnum=0; nodeIDnum<mySHnodeMasterInfo.numNodeIDs; nodeIDnum++)
            {
//                if( shIrqT1thisLevel == mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
                if( LOAD_INTENSITY_FULL_ON >= mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodeLevelCurrent )
                {
                    // turn Triac pulse OFF for ANYTHING, unless it is FULL-ON
                    digitalWrite(mySHnodeMasterInfo.nodeInfo[nodeIDnum].SHthisNodePin, LOAD_POWERED_OFF);
                }
            }

            //shIrqT1thisLevel = 0; // make sure to stay at level 0, no decrement to negative numbers
            shIrqT1thisLevel = LOAD_INTENSITY_MAX; // reset to MAX for next iteration through the set of shIrqT1thisLevel values (next half-period)

            // do NOT restart the timer here, wait for Zero-Cross interrupt to start it during next half-cycle
            
            // fallthrough to default

        default:
            //error - stay stopped from above
            shIrqT1thisLevel = LOAD_INTENSITY_MIN; // reset to MIN for sanity
            break;

        // case value levels

    } // switch

    digitalWrite(pinTimer1irq, LOW);   // sets the pin "off"

    interrupts();
    
} // irqT1triacTriggers()



