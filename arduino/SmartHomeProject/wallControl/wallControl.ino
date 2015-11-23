// Include Arduino standard libraries
#include "Arduino.h"

// Because Unos and Dues have different sizes for int
#include "stdint.h"

#include <Wire.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Adafruit_STMPE610.h"

// include the SD library:
#include <SD.h>


////#include "SmartHome_Zigbee.h"
#include <SmartHome_Zigbee.h>
SHzigbee mySHzigbee = SHzigbee();


#include <SmartHome_NodeInfo.h>
SHnodeInfo thisWCnodeInfo;
SHnodeInfo curLoadNodeInfo;


// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

// For the Adafruit shield, these are the default.
// The display also uses hardware SPI, plus #9 & #10
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);


// The STMPE610 uses hardware SPI on the shield, and #8
#define STMPE_CS 8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);


// set up variables using the SD utility library functions:
#define SD_CS 4
// change this to match your SD shield or module;
const int chipSelect = 4; // for Adafruit 2.8" LCD Resistive Touchscreen

Sd2Card card;
SdVolume volume;
SdFile root;

// SD dir struct
// R  (numerical direcotry name representing an unsigned 16bit int number, room number)
//    L.BIN (numerical filename with .BIN extension represeiting the controllable loads in that room
// number of rooms is determined by dir names of each room number onthe SD card
// each wall controller will dor a dir search to figure that out
// default room is room # 0
uint16_t currentRoomNum = 0;
uint16_t lastRoomNum = 0; // to be updated in setup() bsaed on what is found in SD card
uint8_t  currentLoadNumInRoom = 0;
uint16_t curNumLoadsInRoom= 0;
uint8_t  lastLoadNumInRoom = 0;


//#include "SpiTFTbitmap.h"  // TODO - try to move bmpDraw function stuff into an external library at some point


#define SH_WALLCONTROL_LCD_BACKDROP_X (uint16_t)240
#define SH_WALLCONTROL_LCD_BACKDROP_Y (uint16_t)320


#define YES (uint8_t)0x01
#define NO  (uint8_t)0x00


#if 0
// TODO move to nodeinfo iclude file
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
#define ROOM_NO_LOADS (uint16_t)0xffff
#endif


// SmartHome Node ID for this Wall Control unit
uint16_t thisWCnodeID = 0;
uint8_t *ptrThisWCnodeID = (uint8_t *)&thisWCnodeID;


// Initialize this software program before running the main loop
void setup() {
    // put your setup code here, to run once:
    uint8_t tmpR8 = 0;
    
    Serial.begin(9600);


    // Enable LCD TFT display
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    #define TFT_ROT_PORTRAIT 0
    tft.setRotation(TFT_ROT_PORTRAIT);


    // Enable LCD/TFT Touchscreen touch sensor
    if (!ts.begin()) { 
        Serial.println("Unable to start touchscreen.");
    } 
    else { 
        Serial.println("Touchscreen started."); 
        ts.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
    }


    // Enable SD/microSD card storage
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
        Serial.println("failed!");
    }
    else
    {
        Serial.println("OK!");

        // Read the SmartHome ID for this Wall Control unit from SD card
        //getThisWCnodeIDfromSD();
        thisWCnodeID = getThisWCnodeIDfromSD();
        Serial.print("This WC node has SH nodeID=");
        Serial.println(thisWCnodeID, HEX);
        
        char roomNumDirName[7] = "0";
        char loadNumFileName[16] = "/1/2.BIN";

        volatile uint16_t tmpRoomNum = 0;
            volatile uint16_t tmpLoadNum = 0;
            
        sprintf(roomNumDirName, "/%d", tmpRoomNum);
        while( (tmpRoomNum <= 65535) && SD.exists(roomNumDirName) )
        {
            Serial.print("Checking SD for ");
            Serial.println(roomNumDirName);

            lastRoomNum = tmpRoomNum;
            Serial.print("Found Room # ");
            Serial.println(tmpRoomNum, DEC);
            tmpRoomNum += 1;
            sprintf(roomNumDirName, "/%d", tmpRoomNum);
        }        
          
    } // else SD.begin

     
    //bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, SH_WALLCONTROL_LCD_BACKDROP_X, SH_WALLCONTROL_LCD_BACKDROP_Y);
//    bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, 0, 0, 240, 320);   
    lcdDrawBackDrop();
//    lcdDrawRoomBtn(0);
//    lcdDrawLoadBtn(0, 0);
    lcdDrawLvlIndBar();

//    getCurLvlForLoad(currentRoomNum, DEFAULT_LOAD); // default load for any wall control is Room 0, Load 0 as on SD card
    lcdDrawCurLevel();

#if 0
    lcdDrawLvlIndicator(5);
    lcdDrawLvlIndicator(4);
    lcdDrawLvlIndicator(3);
    lcdDrawLvlIndicator(2);
    lcdDrawLvlIndicator(1);
    lcdDrawLvlIndicator(0);
#endif

    // Select the default room ID as our current room
    selectRoom(DEFAULT_ROOM_NUM);
    
#if 0
    // Attempt to transmit a TX frame for debugging
//    mySHzigbee.initXmitAPIframe(); // init is now done in the SHzigbee class constructor

    // Fill TX frame payload (SH message) with current message values
    mySHzigbee.prepareTXmsg( 
                  (uint16_t)0xabcd,    // SH dest ID
                  (uint16_t)0xf00d,    // SH src ID
                  SH_MSG_TYPE_CMD_REQ, // SH msg Type
                  SH_CMD_LOAD_ON,      // SH command
                  (uint8_t)0x00,       // SH statusH
                  (uint8_t)0x00,       // SH statusL
                  (uint8_t)0x00        // SH statusVal
                );

    // send our prepared ZB TX frame out uart to Xbee module for Zigbee transmit
    mySHzigbee.zbXmitAPIframe();

    Serial.println("");
    Serial.println("");
#endif

//    Serial.println("Exiting setup()");
}


// Look on SD card for number of loads in the given room.
// Each room on SD card is a directory with same name as its room number.
// Each load on SD is a few files with names of load number DOT extension, such as 0.BIN or 1.BIN etc.
// Cycle through the numbers, starting with first load number of 0 and counting up, 
// until an N.BIN file is not found for that eval load number N.
uint16_t getNumLoadsInRoomFromSD(uint16_t roomNum)
{
    uint8_t tmpLoadNum = 0;
    char loadNumFileName[16] = "/65535/255.BIN";  // default longest filename to make sure have enough chars in string

    sprintf(loadNumFileName, "/%d/%d.BIN", roomNum, DEFAULT_LOAD_NUM);
    if( !SD.exists(loadNumFileName) )
    {
        return(ROOM_NO_LOADS);
    }
    
    
    // Get last load number for the default room
    sprintf(loadNumFileName, "/%d/%d.BIN", roomNum, tmpLoadNum);

    while( (tmpLoadNum <= 255) && SD.exists(loadNumFileName) )
    {
        Serial.print("    getNumLoadsInRoomFromSD - Found filename ");
        Serial.print(loadNumFileName);
        Serial.print(" on SD card for ");
        Serial.print(DEFAULT_ROOM_NUM, DEC);
        Serial.print(" / ");
        Serial.print(tmpLoadNum, DEC);
        Serial.println("");
            
        lastLoadNumInRoom = tmpLoadNum;
        tmpLoadNum += 1;            
        sprintf(loadNumFileName, "/%d/%d.BIN", roomNum, tmpLoadNum);
    }

    Serial.print("    getNumLoadsInRoomFromSD - Did NOT find filename ");
    Serial.print(loadNumFileName);
    Serial.print(" on SD card for ");
    Serial.print(DEFAULT_ROOM_NUM, DEC);
    Serial.print(" / ");
    Serial.print(tmpLoadNum, DEC);
    Serial.println("");

    // first is number 0, so add 1 to the last number detected for how many there are
    return(lastLoadNumInRoom+1);  
}


// Get SmartHome ID value for this Wall Control unit from SD card
uint16_t getThisWCnodeIDfromSD(void)
{
    uint16_t tmpWCloadID = 0;
    uint8_t *ptrTmpWCloadID = (uint8_t *)&tmpWCloadID;
    
    #define myIDfilename "/ME.BIN"
    if( SD.exists(myIDfilename) )
    {
        File myIDfile = SD.open(myIDfilename, FILE_READ);
        if( myIDfile.available() )
        {
            //ptrThisWCnodeID[1] = (uint8_t)myIDfile.read();
            ptrTmpWCloadID[1] = (uint8_t)myIDfile.read();
        }
        if( myIDfile.available() )
        {
            //ptrThisWCnodeID[0] = (uint8_t)myIDfile.read();
            ptrTmpWCloadID[0] = (uint8_t)myIDfile.read();
        }
        myIDfile.close();
        

        Serial.print("SmartHome Node ID for this control unit = ");
        //Serial.println(thisWCnodeID, HEX);
        Serial.println(tmpWCloadID, HEX);
    }

    //return(thisWCnodeID)
    return(tmpWCloadID);
}


// Get SmartHome ID value for this Wall Control unit from SD card
uint16_t getLoadNodeIDfromSD(uint16_t roomNum, uint8_t loadNum)
{
    uint16_t tmpLoadID = LOAD_FIRST; 
    uint8_t *prtTmpLoadID = (uint8_t *)&tmpLoadID;
    char loadNumFileName[16] = "/65535/255.BIN";  // default longest filename to make sure have enough chars in string

    sprintf(loadNumFileName, "/%d/%d.BIN", roomNum, loadNum);
    if( SD.exists(loadNumFileName) )
    {
        File loadIDfile = SD.open(loadNumFileName, FILE_READ);
        if( loadIDfile.available() )
        {
            prtTmpLoadID[1] = (uint8_t)loadIDfile.read();
        }
        if( loadIDfile.available() )
        {
            prtTmpLoadID[0] = (uint8_t)loadIDfile.read();
        }
        loadIDfile.close();
        
        Serial.print("SmartHome Node ID for this load unit = ");
        Serial.println(tmpLoadID, HEX);
    }

    return(tmpLoadID);
}


// Get SmartHome Load Type value for this Wall Control unit from SD card
uint8_t getLoadTypefromSD(uint16_t roomNum, uint8_t loadNum)
{
    uint8_t tmpLoadType = NODEINFO_NODETYPE_LIGHT; //loads are not controls, and more loads will be lights than fans
    char loadNumFileName[16] = "/65535/255.BIN";  // default longest filename to make sure have enough chars in string

    sprintf(loadNumFileName, "/%d/%d.BIN", roomNum, loadNum);
    if( SD.exists(loadNumFileName) )
    {
        File loadIDfile = SD.open(loadNumFileName, FILE_READ);
        if( loadIDfile.available() )
        {
            loadIDfile.read(); // read but skip over load ID H byte here
        }
        if( loadIDfile.available() )
        {
            loadIDfile.read();  // read but skip over load ID L byte here
        }
        if( loadIDfile.available() )
        {
            tmpLoadType = (uint8_t)loadIDfile.read();  // skip over load ID L byte here
        }
        loadIDfile.close();
        
        Serial.print("SmartHome Node type for this load unit = ");
        Serial.println(tmpLoadType, HEX);
    }

    return(tmpLoadType);
}


// return if SmartHome load exists (the BIN file for it was found on SD card)
boolean loadFileExists(uint16_t roomNum, uint8_t loadNum)
{
    char loadNumFileName[16] = "/65335/255.BIN";

    
    //sprintf(loadNumFileName, "/%d/%d.BIN", roomNum, roomNum);
    sprintf(loadNumFileName, "%d/%d.BIN", roomNum, roomNum);

    if( SD.exists(loadNumFileName) )
    {
        Serial.print("    Found filename ");
        Serial.print(loadNumFileName);
        Serial.print(" on SD card");
        return(true);
    }

    return(false);
}


#define TS_DEBOUNCE_MILLIS  (uint32_t)2500  // number of milliseconds to delay for touchscreen debounce
// These need to be GLOBAL vars. If define then insid eloop, then they get reinitialized to 0,0,500 each iteration
// and that breaks the debounce checking quite badly!
volatile uint32_t prevMillis = 0;
volatile uint32_t nowMillis = 0;
volatile uint32_t nextMillis = TS_DEBOUNCE_MILLIS;

void loop() {
    uint16_t x, y = 0;
    TS_Point p; // Point struct for Touchscreen events

  // put your main code here, to run repeatedly:

    // See if there's any  touch data for us
    //if ( ts.touched() )   // <-- do NOT use this one!! More bogus/invalid events triggered
    if (!ts.bufferEmpty())  // <-- DO use this one!! Cleaner things happening
    {

        // Retrieve a point. This may be legitimate or bogus to be debounced
        p = ts.getPoint();

        // get current time in milliseconds since starting the program
        nowMillis = millis();

        // TODO Add to this for rollover checking, but careful and test a lot as it's tricky
        if( nowMillis >= nextMillis )
        {
            //Serial.println("Processing the TS event");
            
            prevMillis = nowMillis;
            nextMillis = nowMillis + TS_DEBOUNCE_MILLIS;         
        
            // respond to touchscreen input

            // Scale from ~0->4000 to tft.width using the calibration #'s
            p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
            p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

            // Get x/y coord for our chosen rotation of the image
            x = p.x;
            y = p.y;

#if 0
            // print to serial monitor the coordinate of the current touch event
            Serial.print("Have Touch at x=");
            Serial.print(x, DEC);
            Serial.print(" ; y=");
            Serial.println(y, DEC);
#endif
        
            // Figure out which LCD GUI button was pressed and respond accodringly
            SHdoTouchButton(x, y);
        }
        else
        {
//            Serial.println("Debouncing/skipping this TS event");
        }

    }
    #if 0
    else
    {
        //NO touchscreen input for this iteration of loop()
    }
    #endif
    // Check in on SmartHome Zigbee messaging state machine for anything to do goober
    else if(curLoadNodeInfo.newSHmsgTX == YES)
    {
        doWCnodeIDmsgSM();
    }
}


uint8_t prevSHcmd = SH_CMD_NOP;
uint8_t curSHcmd = SH_CMD_NOP;
uint8_t curSHcmdRepeats = 0;
uint16_t prevLoadID = 0;
uint16_t curLoadID = 0;

// Figure out which LCD GUI button was pressed and respond accodringly
void SHdoTouchButton(uint16_t x, uint16_t y)
{
    Serial.print("Touch Button ");  

    // button locations on-screen
    // FAV 45:55   ; 125:107
    // INC 45:10   ; 125:45
    // DEC 45:125  ; 125:160
    // ON  140:10  ; 230:70
    // OFF 140:100 ; 230:160

    #define NUM_CMD_REPEATS_SAVE_FAV 2
    #define FAKE_FAV_VALUE_DEBUG 4
    

    // save current load ID and SH command for comparison next time through to detect repeats
    prevLoadID  = curLoadID;
    curLoadID = curLoadNodeInfo.SHthisNodeID;

    // The X and Y coordinates checked here were measured with the intended wall-switch background image displayed, and touching the touchscreen with a stylus
    // and noting the coordinate numbers coming out of the "TouchTest" Example program included with the touch-sensor driver library for this panel.
    if( x>=140 && x<230)
    {
        if(y>10 && y<70)  // ON button
        {
            Serial.println("ON");             

            if(  SH_POWERED_OFF == curLoadNodeInfo.SHthisNodeIsPowered )
            {
                curLoadNodeInfo.SHthisNodeIsPowered = SH_POWERED_ON;
                curLoadNodeInfo.SHthisNodeLevelCurrent = curLoadNodeInfo.SHthisNodeMsg.SHstatusVal;

                // Fill TX frame payload (SH message) with current message values
                curLoadNodeInfo.SHthisNodeMsg.SHothrID = thisWCnodeID;
                curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_CMD_REQ;
                curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_ON;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusH = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusL = curLoadNodeInfo.SHthisNodeIsPowered;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = curLoadNodeInfo.SHthisNodeLevelCurrent;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved1 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved2 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;

                // Set the xmitReady flag in the nodeinfo structure set above
                curLoadNodeInfo.newSHmsgTX = YES;
                curLoadNodeInfo.SHmsgCurrentState = SH_MSG_ST_CMD_INIT;
            }
        }
        else if(y>100 && y<160)  // OFF button
        {
            Serial.println("OFF"); 

            if(  SH_POWERED_ON == curLoadNodeInfo.SHthisNodeIsPowered )
            {
                curLoadNodeInfo.SHthisNodeIsPowered = SH_POWERED_OFF;
                curLoadNodeInfo.SHthisNodeLevelCurrent = curLoadNodeInfo.SHthisNodeMsg.SHstatusVal;

                // Fill TX frame payload (SH message) with current message values
                curLoadNodeInfo.SHthisNodeMsg.SHothrID = thisWCnodeID;
                curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_CMD_REQ;
                curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_OFF;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusH = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusL = SH_POWERED_OFF;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = curLoadNodeInfo.SHthisNodeLevelCurrent;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved1 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved2 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;

                // Set the xmitReady flag in the nodeinfo structure set above
                curLoadNodeInfo.newSHmsgTX = YES;
                curLoadNodeInfo.SHmsgCurrentState = SH_MSG_ST_CMD_INIT;
            }
        }
        if(y>180 && y<220)  // Room -> button (select next Room)
        {
            Serial.println("Room ->"); 
            changeRoom(ROOM_CHANGE_ROTR);
        }
        else if(y>240 && y<290)  // Load -> button (select next Load)
        {
            Serial.println("Load ->"); 
            changeLoad(LOAD_CHANGE_ROTR);
        }
    }
    else if( (x>=10 && x<100) && (y>180 && y<220) )  // Room <- button (select previous Room)
    {
            Serial.println("Room <-"); 
            changeRoom(ROOM_CHANGE_ROTL);
    }
    else if( (x>=10 && x<100) && (y>240 && y<290) )  // Load <- button (select previous Load)
    {
            Serial.println("Load <-"); 
            changeLoad(LOAD_CHANGE_ROTL);
    }
    else if( x>=45 && x<125) 
    {
        if(y>10 && y<45)  // Up-Arrow button (increase load intensity)
        {
            Serial.println("Increase");             

            if( 5 > (curLoadNodeInfo.SHthisNodeLevelCurrent) )
            {
                // increment the current intensitylevel value
                curLoadNodeInfo.SHthisNodeLevelCurrent += 1;
                curLoadNodeInfo.SHthisNodeIsPowered = SH_POWERED_ON;

                // Fill TX frame payload (SH message) with current message values
                curLoadNodeInfo.SHthisNodeMsg.SHothrID = thisWCnodeID;
                curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_CMD_REQ;
                curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_INC;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusH = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusL = curLoadNodeInfo.SHthisNodeIsPowered;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = curLoadNodeInfo.SHthisNodeLevelCurrent;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved1 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved2 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;

                // Set the xmitReady flag in the nodeinfo structure set above
                curLoadNodeInfo.newSHmsgTX = YES;
                curLoadNodeInfo.SHmsgCurrentState = SH_MSG_ST_CMD_INIT;
            }
            else
            {
                // do nothing, cannot increase beyond maximum
            }
        }
        else if(y>60 && y<110)  // FAV button (favorite intensity value for this load)
        {
            Serial.println("FAV"); 

            curLoadNodeInfo.SHthisNodeIsPowered = SH_POWERED_ON;

            // Fill TX frame payload (SH message) with current message values
            curLoadNodeInfo.SHthisNodeMsg.SHothrID = thisWCnodeID;
            curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_CMD_REQ;

            // if pressed FAV 4 times in a row then that means to SAVE the current intensity as the new favorite for the selected load
            // this is done in the do*SM function based on prevSHcmd and curSHcmd and curSHcmdRepeats
            // if pressed FAV 4 times in a row then that means to SAVE the current intensity as the new favorite for the selected load
            if( ( prevSHcmd == SH_CMD_LOAD_GOTOFAV) && (prevLoadID == curLoadNodeInfo.SHthisNodeID) && (NUM_CMD_REPEATS_SAVE_FAV >= curSHcmdRepeats) )
            {
                curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_SAVEFAV;
                curSHcmdRepeats = 0;
            }
            else
            {
                curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_GOTOFAV;
                curSHcmdRepeats += 1;
            }

            curLoadNodeInfo.SHthisNodeMsg.SHstatusH = FAKE_FAV_VALUE_DEBUG;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusL = curLoadNodeInfo.SHthisNodeIsPowered;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = FAKE_FAV_VALUE_DEBUG; // TODO - no hardcode test value
            curLoadNodeInfo.SHthisNodeMsg.SHreserved1 = SH_RESERVED_BYTE;
            curLoadNodeInfo.SHthisNodeMsg.SHreserved2 = SH_RESERVED_BYTE;
            curLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;

            // Set the xmitReady flag in the nodeinfo structure set above
            curLoadNodeInfo.newSHmsgTX = YES;
            curLoadNodeInfo.SHmsgCurrentState = SH_MSG_ST_CMD_INIT;

        }
        else if(y>125 && y<160)  // Down-Arrow button (decrease load intensity)
        {
            Serial.println("Decrease"); 

            if( 0 < (curLoadNodeInfo.SHthisNodeLevelCurrent) ) //&& ( SH_POWERED_ON == curLoadNodeInfo.SHthisNodeIsPowered) )
            {
                // decrement the current intensitylevel value
                curLoadNodeInfo.SHthisNodeLevelCurrent -= 1;

                if(curLoadNodeInfo.SHthisNodeLevelCurrent == 0)
                {
                    curLoadNodeInfo.SHthisNodeIsPowered = SH_POWERED_OFF;
                }
                else
                {
                    curLoadNodeInfo.SHthisNodeIsPowered = SH_POWERED_ON;
                }

                // Fill TX frame payload (SH message) with current message values
                curLoadNodeInfo.SHthisNodeMsg.SHothrID = thisWCnodeID;
                curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_CMD_REQ;
                curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_DEC;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusH = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusL = curLoadNodeInfo.SHthisNodeIsPowered;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = curLoadNodeInfo.SHthisNodeLevelCurrent;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved1 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHreserved2 = SH_RESERVED_BYTE;
                curLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
                curLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;

                // Set the xmitReady flag in the nodeinfo structure set above
                curLoadNodeInfo.newSHmsgTX = YES;
                curLoadNodeInfo.SHmsgCurrentState = SH_MSG_ST_CMD_INIT;
            }
            else
            {
                // do nothing, can't decrease below minimum/OFF
//                curLoadNodeInfo.SHthisNodeMsg.SHstatusL = SH_POWERED_OFF;
            }
        }
    }    

    prevSHcmd = curLoadNodeInfo.SHthisNodeMsg.SHcommand;

    Serial.println("");
}


// print to serial monitor the invalid LCD touchscreen button coordinate
void printInvalidCoord(uint16_t x, uint16_t y)
{
    Serial.print("INVALID coordinate (no button at) ");
    Serial.print(x, DEC);
    Serial.print(" : ");
    Serial.print(y, DEC);
}


// Read the Load ID information from SD card for the given room number and load number
// The 16bit ID is the first two bytes of a load's #.BIN file
uint16_t getLoadIDfromSD(uint16_t roomNum, uint8_t loadNum)
{
    uint16_t tmpLoadID = 0;
    uint8_t *ptrTmpLoadID = (uint8_t *)&tmpLoadID;
    char loadNumFileName[16] = "/1/2.BIN";

    // TODO add code to get ID from SD card, this is DEV/DEBUG only
    tmpLoadID = SH_LOAD_ID_ROOM0_LOAD0 ;

    sprintf(loadNumFileName, "/%d/%d.BIN", roomNum, loadNum);
    
    if( SD.exists(loadNumFileName) )
    {
        File loadIDfile = SD.open(loadNumFileName, FILE_READ);
        if( loadIDfile.available() )
        {
            ptrTmpLoadID[1] = (uint8_t)loadIDfile.read();
        }
        if( loadIDfile.available() )
        {
            ptrTmpLoadID[0] = (uint8_t)loadIDfile.read();
        }
        loadIDfile.close();
        
        Serial.print("SmartHome Node ID for Room ");
        Serial.print(roomNum, DEC);
        Serial.print(" and Load ");
        Serial.print(loadNum, DEC);
        Serial.print(" is ");
        Serial.println(tmpLoadID, HEX);        
    }
    
    return(tmpLoadID);
}


// Send SmartHome message via Zigbee to request the current level for the given load ID
uint8_t getCurLvlForLoad(uint16_t loadID)
{
    // TODO
    return(3);
}


// Draw the LCD GUI backdrop image, which makes up most of the screen. 
// Other image objects are painted over top of this.
void lcdDrawBackDrop(void)
{
    //#define SH_WALLCONTROL_LCD_BACKDROP_FILE "/IMAGES/BACKDROP.BMP"
    #define SH_WALLCONTROL_LCD_BACKDROP_FILE "/BCKDRP24.BMP"
  
    //bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, SH_WALLCONTROL_LCD_BACKDROP_X, SH_WALLCONTROL_LCD_BACKDROP_Y);
    bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, 0, 0, 240, 320);
}


// Draw the current intensity level for the currently selected load
void lcdDrawCurLevel(void)
{
    // TODO - check current level for the currently selected load
    lcdDrawLvlIndicator(3);
}


// Draw the vertical bar portion of the level indicator
void lcdDrawLvlIndBar(void)
{
    #define FILENAME_LVL_IND_BAR "/INDBAR.BMP"

    #if 0
    Serial.print("Drawing Indicator Bar");
    Serial.print(" using filename ");
    Serial.println(FILENAME_LVL_IND_BAR);
    #endif
    
//    bmpDraw(FILENAME_LVL_IND_BAR, 118, 12, 240, 320);  // for about center of horizontal dimension
    bmpDraw(FILENAME_LVL_IND_BAR, 0, 12, 240, 320);      // for left side of horizontal dimension
}


// Draw the load level setting indicator bob graphic to screen.
// at an appropriate coordinate location for the given level.
// This will redraw the level indicator bar to clear the 
// previous bob position.
void lcdDrawLvlIndicator(uint8_t myLevel)
{
    // Redraw the Level indicator bar to clear out the current/previous level drawing position
    lcdDrawLvlIndBar();
    
    #define FILENAME_LVL_IND "/LVLIND.BMP"

    #if 0
    Serial.print("Drawing level indicator bob using filename ");
    Serial.println(FILENAME_LVL_IND);
    #endif

    switch(myLevel)
    {
        case (uint8_t)5: // Full ON
            bmpDraw(FILENAME_LVL_IND, 0, 17, 240, 320);
            break;

        case (uint8_t)4: 
            bmpDraw(FILENAME_LVL_IND, 0, 44, 240, 320);
            break;

        case (uint8_t)3: 
            bmpDraw(FILENAME_LVL_IND, 0, 70, 240, 320);
            break;

        case (uint8_t)2: 
            bmpDraw(FILENAME_LVL_IND, 0, 96, 240, 320);
            break;

        case (uint8_t)1: 
            bmpDraw(FILENAME_LVL_IND, 0, 123, 240, 320);
            break;

        case (uint8_t)0: // Full OFF
            bmpDraw(FILENAME_LVL_IND, 0, 150, 240, 320);
            break;

        default:
            // Do nothing
            break;
    }        
}


// Draw the SmartHome GUI Room Button for the given room number.
// The graphic for this button is stored on and retrieved from SD card.
// The room number alone is enough to get the correct BMP file from SD.
void lcdDrawRoomBtn(uint16_t roomNum)
{
    char btnFileName[9];

    sprintf(btnFileName, "/%d/R.BMP", roomNum);

    #if 0
    Serial.print("Drawing Button graphic for room ");
    Serial.print(roomNum, DEC);
    Serial.print(" using filename ");
    Serial.println(btnFileName);
    #endif    

    bmpDraw(btnFileName, 0, 191, 240, 320);
}

// /65536/255.bmp$
void lcdDrawLoadBtn(uint16_t roomNum, uint8_t loadNum)
{
    char btnFileName[16];

    sprintf(btnFileName, "/%d/%d.BMP", roomNum, loadNum);

    #if 0
    Serial.print("Drawing Button graphic for room ");
    Serial.print(roomNum, DEC);
    Serial.print(" using filename ");
    Serial.println(btnFileName);
    #endif
    
    bmpDraw(btnFileName, 0, 256, 240, 320);
}





#define USE_BMP_DRAW
#ifdef USE_BMP_DRAW
// This ifdef block is used to indicate the bounds of the bmpDraw code
// used from the spitftbitmap example program as provided with the
// Adafruit IL9341 driver library for their V2 2.8inch LCD/TFT touchscreen Arduino shield

/***************************************************
  This is our Bitmap drawing example for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
 
// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

// Bill Toner modified this to rename x and y to x1 and y1 and then add tftW and tftH
// parameters. The x1 and y1 renames are probably not necessary after realizing that
// what I tried to do for x2 and y2 didn't work, and renamed tose to tftW and tftH
// and better code based on those works better.

// This "library" function does NOT work well if x1 != 0, so I redrew my 
// initial graphics to rearange buttons on screen, so that the level indicator
// can now be drawn at left side, to get x1=0, so that redrawign that works OK.

// Bill Toner - I added this, but interestingly, if DEBUG_BMP_DRAW is commented out/disabled, then no picture appears on the LCD screen. 
// So please UNcomment this always, unless corrected. Seemed like a nice addition as I get a lot of Serial output.
#define DEBUG_BMP_DRAW  // UNcomment if want debug strings sent to Serial Monitor, comment out if do not want

#define BUFFPIXEL 20

void bmpDraw(char *filename, uint16_t x1, uint16_t y1, uint16_t tftW, uint16_t tftH) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

//  if((x >= tft.width()) || (y >= tft.height())) return;

  #ifdef DEBUG_BMP_DRAW
      Serial.println();
      Serial.print(F("Loading image '"));
      Serial.print(filename);
      Serial.println('\'');
  #endif
  
  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    #ifdef DEBUG_BMP_DRAW
        Serial.print(F("File not found"));
    #endif
    
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    #ifdef DEBUG_BMP_DRAW
        Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    #endif
    
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data

    #ifdef DEBUG_BMP_DRAW
        Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    #endif
    
    // Read DIB header
    #ifdef DEBUG_BMP_DRAW
        Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    #endif
    
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel

      #ifdef DEBUG_BMP_DRAW
          Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      #endif
      
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!

        #ifdef DEBUG_BMP_DRAW
            Serial.print(F("Image size: "));
            Serial.print(bmpWidth);
            Serial.print('x');
            Serial.println(bmpHeight);
        #endif
        
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
//        if((x+w-1) >= tft.width())  w = tft.width() - x;
        if((x1+w-1) >= tftW)  w = tftW - x1 ;

//        if((y+h-1) >= tft.height()) h = tft.height() - y;
        if((y1+h-1) >= tftH) h = tftH - y1 ;

        // Set TFT address window to clipped image bounds
//        tft.setAddrWindow(x, y, x+w-1, y+h-1);
        tft.setAddrWindow(x1, y1, x1+w-1, y1+h-1);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

//          for (col=0; col<w; col++) { // For each pixel...
          for (col=x1; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.color565(r,g,b));
          } // end pixel
        } // end scanline
        
        #ifdef DEBUG_BMP_DRAW
            Serial.print(F("Loaded in "));
            Serial.print(millis() - startTime);
            Serial.println(" ms");
        #endif
        
      } // end goodBmp
    }
  }

  bmpFile.close();
  
  #ifdef DEBUG_BMP_DRAW
      if(!goodBmp) Serial.println(F("BMP format not recognized."));
  #endif
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

#endif // USE_BMP_DRAW





// Do a SmartHome message conversation over Zigbee, as appropriate for the current message command
// This is Wall-Control unit specific, a Load-Driver unit will have it's own implementation of something similar
// Control units and Load Driver units will be opposits in if they are RX or TX in each state
void doWCnodeIDmsgSM(void) //uint8_t nodeInfoIndex)
{
    volatile uint8_t   SHmsgNextState;         // which of 4 message stages are we in now, or 0=idle?
    volatile uint8_t   SHmsgCmd;
    volatile uint8_t   SHmsgStatus;

    
    // update to previous iteration's next state
    curLoadNodeInfo.SHmsgCurrentState = curLoadNodeInfo.SHmsgNextState;


    #if 1
    Serial.print("Entering doWCnodeIDmsgSM - newSHmsgTX = ");
    Serial.print(curLoadNodeInfo.newSHmsgTX, HEX);
    Serial.print(" ; SHmsgCurrentState= ");
    Serial.print(curLoadNodeInfo.SHmsgCurrentState, HEX);
    Serial.print(" ; SHthisNodeMsg.SHcommand = ");
    Serial.print(curLoadNodeInfo.SHthisNodeMsg.SHcommand, HEX);
    Serial.println();
    #endif


    switch ( curLoadNodeInfo.SHmsgCurrentState )
    {
        // Linux Server will receive ALL messages, and log ALL messages. No need to detect errors and specifically tell server of them.
//for Linux server logging - use dir name matching node ID with files inside to give room location etc. info

        case SH_MSG_ST_IDLE:  // Doing nothing
            Serial.println("    In SH_MSG_ST_IDLE, doing nothing");

        case SH_MSG_ST_CMD_INIT: // WC TX - send a new SmartHome command message to a load
            if ( SH_MSG_TYPE_CMD_REQ == curLoadNodeInfo.SHthisNodeMsg.SHmsgType )
            {
              Serial.print("    Sending a SH_MSG_TYPE_CMD_REQ to load ID ");
              //Serial.print(curLoadNodeInfo.SHthisNodeID, HEX);
              Serial.println();
              
                // Prepare and Send the appropriate SmartHome command message via Zigbee
                mySHzigbee.prepareTXmsg( 
                    curLoadNodeInfo.SHthisNodeID,              // DestID is the load target that this WC node wants to control
                    thisWCnodeID,                              // Src ID is this node
                    SH_MSG_TYPE_CMD_REQ,                       // MsgType
                    curLoadNodeInfo.SHthisNodeMsg.SHcommand,   // CMD
                    curLoadNodeInfo.SHthisNodeMsg.SHstatusH,   // Status H byte
                    curLoadNodeInfo.SHthisNodeMsg.SHstatusL,   // Status L byte
                    curLoadNodeInfo.SHthisNodeMsg.SHstatusVal  // Status value
                );

                mySHzigbee.zbXmitAPIframe();      // Transmit the SmartHome message over Zigbee
                curLoadNodeInfo.newSHmsgTX = NO;  // sent it, so make sure we don't send it again

//                curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_ACK_REQ;
//                curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
                curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_COMPLETE;

                // TODO remove dev hack, reload this same function after hackign to the COMPLETE state to avoid dogin that next transmit, we want it in this one
                doWCnodeIDmsgSM();
            }
            else
            {
                // nothing to do at this time
                curLoadNodeInfo.newSHmsgTX = NO; 
                curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
            }
            break;



        case SH_MSG_ST_ACK_REQ:  // WC RX - Wait for ACK_REQ from the node we are talking to
            Serial.println("    Received a SH_MSG_ST_ACK_REQ");
#if 0
            // for WC unit we RX instead fo TX as coded for Load Driver
            curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = TXmsgACKREQ();
            if ( curLoadNodeInfo.SHthisNodeMsg.SHstatusTX == SH_STATUS_SUCCESS )
            {
                curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_CNFRM;
            }
            else
            {
                curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
            }
#endif
            curLoadNodeInfo.newSHmsgRX = NO;  
            curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
            break;

        case SH_MSG_ST_CNFRM:  // WC TX - tell other node if we initiated command request CMD_REQ to it or not
            Serial.println("    (Maybe) Sending a SH_MSG_ST_CNFRM");
            curLoadNodeInfo.newSHmsgTX = NO;  
            curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
            break;

        case SH_MSG_ST_COMPLETE: // WC RX - wait for Load Driver to tell how command execution went
//            Serial.print("    Received a SH_MSG_ST_COMPLETE ; SHcommand = ");
            Serial.print("    State SH_MSG_ST_COMPLETE - Received SHcommand = ");
            Serial.print(curLoadNodeInfo.SHthisNodeMsg.SHcommand, HEX);
            Serial.println();

            if(1) //received an appropriate confirmation message
            {
                switch( curLoadNodeInfo.SHthisNodeMsg.SHcommand )
                {
                    // These commands will redraw the wall control Intensity Level Indicator based on value in statusVal field
                    case SH_CMD_LOAD_READCRNT:
                    case SH_CMD_LOAD_GOTOFAV: // implies will be powered, as FAV cannot be full-off level
                        curLoadNodeInfo.SHthisNodeLevelFav = curLoadNodeInfo.SHthisNodeMsg.SHstatusH;
                        curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = curLoadNodeInfo.SHthisNodeMsg.SHstatusH;

                        // NO break here so we also get the Powered and LevelCurrent values below
                    
                    case SH_CMD_LOAD_ON:
                    case SH_CMD_LOAD_OFF:
                    case SH_CMD_LOAD_TOGLPWR:
                        curLoadNodeInfo.SHthisNodeIsPowered = curLoadNodeInfo.SHthisNodeMsg.SHstatusL;
                        curLoadNodeInfo.SHthisNodeLevelCurrent = curLoadNodeInfo.SHthisNodeMsg.SHstatusVal;

                    case SH_CMD_LOAD_INC:
                    case SH_CMD_LOAD_DEC:
                
                        if(SH_POWERED_ON == curLoadNodeInfo.SHthisNodeMsg.SHstatusL)
                        {
                            // lcd draw current intensity level to the status val returned in this message
                            lcdDrawLvlIndicator(curLoadNodeInfo.SHthisNodeMsg.SHstatusVal);
                        }
                        else
                        {
                            // NOT currently powered, so lcd draw level indicator as lowest level (OFF), regardless of current intensity level reported by the load
                            lcdDrawLvlIndicator(0);
                        }

                        break;

                    default:
                        // do nothing for other command types coming back
                        break;
                }

                curLoadNodeInfo.newSHmsgRX = NO;  

                // reset back to idle state
                curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;  
                curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_TYPE_IDLE;          
            }
            break;

        default: // RX or TX of something unknown
            // invalid next message state, reset to idle, careful about stranding another node mid-conversation.
            curLoadNodeInfo.SHmsgNextState = SH_MSG_ST_IDLE;
            break;
    }
}


// Prepare a TX Zigbee frame to send a SmartHome ACK/REQ message back to the node that initiated a new command
uint8_t TXmsgACKREQ(uint8_t nodeInfoIndex)
{
#if 0
  prepareTXmsg( 
                curLoadNodeInfo.SHthisNodeMsg.SHothrID,   // DestID is node that initiated this conversation
                curLoadNodeInfo.SHthisNodeID,             // Src ID is this node
                SH_MSG_TYPE_ACK_CREQ,                                                // MsgType
                curLoadNodeInfo.SHthisNodeMsg.SHcommand,  // CMD
                0, 0, 0                                   // statusH, statusL, statusVal
              );

  Serial.print("nodeID=");
  Serial.print(curLoadNodeInfo.SHthisNodeID, HEX);
  Serial.print(" sending ACK/REQ to ");
  Serial.print(curLoadNodeInfo.SHthisNodeMsg.SHothrID, HEX);
  Serial.print(" ; msgType=");
  Serial.print(curLoadNodeInfo.SHthisNodeMsg.SHmsgType, HEX);
  Serial.print(" ; CMD=");
  Serial.println(curLoadNodeInfo.SHthisNodeMsg.SHcommand, HEX);

  // indicate main loop that a TX frame is ready to send
  newFrameForTX = YES;
  zbXmitAPIframe();
  newFrameForTX = NO;

  return (SH_STATUS_SUCCESS);
#endif

    return (SH_STATUS_SUCCESS);
}


// Read some data for this target load driver node from local SD card based on given room/load numbers
// Ask the load via Zigbee messages other info
// use global var curLoadNodeInfo to store this info for later use
void initCurLoadNodeInfo(uint16_t roomNum, uint8_t loadNum)
{
//    #define NODE_INFO_NODE_IDX 0;  // emulate load driver structures to allow reuse of code

    // make sure not ready to transmit a Zigbee frame yet, have not received a Zigbee frame yet
    curLoadNodeInfo.newSHmsgTX = NO;
    curLoadNodeInfo.newSHmsgRX = NO;
    
    //// get load ID and type from local SD card
    curLoadNodeInfo.SHthisNodeID = getLoadNodeIDfromSD(roomNum, loadNum);
    curLoadNodeInfo.SHthisNodeLoc = roomNum;
//    curLoadNodeInfo.SHothrNodeID = tmpLoadID; // ID of this Wall Control unit, as sender of SH Zigbee messages
    curLoadNodeInfo.SHthisNodeType = getLoadTypefromSD(roomNum, loadNum);
    curLoadNodeInfo.SHthisNodePin = 0;  // NodePin is only used it the load driving unit software, NOT used at Wall Control units software here
    
    // initial values when booting with default load or changing to a different load
    curLoadNodeInfo.SHmsgStatus= SH_STATUS_SUCCESS;


    // SH message payload fields
    curLoadNodeInfo.SHthisNodeMsg.SHothrID = thisWCnodeID;
    curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_ST_IDLE;
    curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_NOP;
    curLoadNodeInfo.SHthisNodeMsg.SHstatusH = 0;
    curLoadNodeInfo.SHthisNodeMsg.SHstatusL = 0;
    curLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;
    curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = 0;
    curLoadNodeInfo.SHthisNodeMsg.SHreserved1 = SH_RESERVED_BYTE;
    curLoadNodeInfo.SHthisNodeMsg.SHreserved2 = SH_RESERVED_BYTE;
    curLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
    curLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
    curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
    curLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;

    
    //// get info from the load itself via Zigbee
    curLoadNodeInfo.SHmsgCurrentState = SH_MSG_TYPE_IDLE;
    curLoadNodeInfo.SHmsgNextState = SH_MSG_TYPE_IDLE;
    
    curLoadNodeInfo.SHmsgCmd = SH_CMD_LOAD_READCRNT;    // Read current values from load, including level, powered, ?Favorite?
    curLoadNodeInfo.newSHmsgTX = YES;
//    doWCnodeIDmsgSM();
//    curLoadNodeInfo.newSHmsgTX = NO;
//    curLoadNodeInfo.SHthisNodeIsPowered = curLoadNodeInfo.SHthisNodeMsg.SHstatusVal; // from Zigbee messaging, FROM the load itself

#if 0
    curLoadNodeInfo.SHmsgCmd = SH_CMD_LOAD_READFAV;    // default to No OPeration for SmartHome command
    curLoadNodeInfo.newSHmsgTX = YES;
    doWCnodeIDmsgSM();
    curLoadNodeInfo.newSHmsgTX = NO;
    curLoadNodeInfo.SHthisNodeLevelFav = curLoadNodeInfo.SHthisNodeMsg.SHstatusVal; // from Zigbee messaging, FROM the load itself

    curLoadNodeInfo.SHmsgCmd = SH_CMD_LOAD_READPWR;    // default to No OPeration for SmartHome command
    curLoadNodeInfo.newSHmsgTX = YES;
    doWCnodeIDmsgSM();
    curLoadNodeInfo.newSHmsgTX= NO;
    curLoadNodeInfo.SHthisNodeIsPowered = curLoadNodeInfo.SHthisNodeMsg.SHstatusVal; // from Zigbee messaging, FROM the load itself
#endif

}


// The "right" button (->) increments the room number
// wrap back to room 0 if currently at last room
// The "left" button (<-) decrements the room number
// wrap back to last room if currently at room 0
void changeRoom(uint8_t changeDirection)
{
    uint16_t newRoom = ROOM_FIRST;

    if(changeDirection == ROOM_CHANGE_ROTR)  // Rotate Right == Increment with wraparound back to 0
    {
        if(currentRoomNum == lastRoomNum)
        {
            newRoom = ROOM_FIRST;
        }
        else
        {
            newRoom = currentRoomNum + 1;
        }
    }
    else if(changeDirection == ROOM_CHANGE_ROTL) // Rotate Left == Decrement with wraparound back to lsat room
    {
        if(currentRoomNum == ROOM_FIRST)
        {
            newRoom = lastRoomNum;
        }
        else
        {
            newRoom = currentRoomNum - 1;
        }
    }

    selectRoom(newRoom);
}


void selectRoom(uint16_t roomNum)
{
    uint16_t tmpNumLoadsInRoom = 0;  

    if( (roomNum >= 0) && (roomNum <= lastRoomNum) )
    {
        Serial.print("Selecting ");
        Serial.print(roomNum, DEC);
        Serial.println(" as current room");

        tmpNumLoadsInRoom = getNumLoadsInRoomFromSD(roomNum);

        if( tmpNumLoadsInRoom == ROOM_NO_LOADS)
        {
            // ERROR, no loads in this room to drive
            Serial.print("Error - NO loads found in room ");
            Serial.println(roomNum, DEC);
        }
        else
        {
            // update room number
            currentRoomNum = roomNum;
            curNumLoadsInRoom = tmpNumLoadsInRoom;        }      
            lastLoadNumInRoom = curNumLoadsInRoom - 1;
            currentLoadNumInRoom = DEFAULT_LOAD_NUM;
//            curLoadID = getLoadIDfromSD(roomNum, currentLoadNumInRoom);        
            curLoadNodeInfo.SHthisNodeType = getLoadTypefromSD(roomNum, currentLoadNumInRoom);
            
            Serial.print("curNumLoadsInRoom = ");
            Serial.println(curNumLoadsInRoom, DEC);

            Serial.print("Current Room = ");
            Serial.print(currentRoomNum, DEC);
            Serial.print(" ; Last Room = ");
            Serial.print(lastRoomNum, DEC);
            Serial.print(" ; Current Load = ");
            Serial.print(currentLoadNumInRoom, DEC);
            Serial.print(" ; Last Load = ");
            Serial.print(lastLoadNumInRoom, DEC);
            Serial.println("");

            lcdDrawRoomBtn(roomNum);

            selectLoad(DEFAULT_LOAD_NUM);
    }
    else
    {
        Serial.print("ERROR - Invalid room number ");
        Serial.println(roomNum, DEC);
    }
}


// The "right" button (->) increments the load number
// wrap back to room 0 if currently at load room
// The "left" button (<-) decrements the load number
// wrap back to last room if currently at load 0
void changeLoad(uint8_t changeDirection)
{
    uint16_t newLoad = DEFAULT_LOAD_NUM;

    if(curNumLoadsInRoom == 1)
    {
        // if only have one load in this room then no change from current load number
        newLoad = currentLoadNumInRoom;  
    }
    else if(changeDirection == LOAD_CHANGE_ROTR)  // Rotate Right == Increment with wraparound back to 0
    {
        if(currentLoadNumInRoom == lastLoadNumInRoom)
        {
            newLoad = LOAD_FIRST;
        }
        else
        {
            newLoad = currentLoadNumInRoom + 1;
        }
    }
    else if(changeDirection == LOAD_CHANGE_ROTL) // Rotate Left == Decrement with wraparound back to last load
    {
        if(currentLoadNumInRoom == LOAD_FIRST)
        {
            newLoad = lastLoadNumInRoom;
        }
        else
        {
            newLoad = currentLoadNumInRoom - 1;
        }
    }

    selectLoad(newLoad);
}


void selectLoad(uint16_t loadNum)
{
    if( (loadNum >= 0) && (loadNum <= lastLoadNumInRoom) )
    {
        // update load number and type
        currentLoadNumInRoom = loadNum;

        curLoadNodeInfo.SHthisNodeLoc  = currentRoomNum;
        curLoadNodeInfo.SHthisNodeType = getLoadTypefromSD(currentRoomNum, loadNum);
        curLoadNodeInfo.SHthisNodeID   = getLoadNodeIDfromSD(currentRoomNum, loadNum);

            curLoadNodeInfo.SHthisNodeMsg.SHothrID = thisWCnodeID; // this wall control is the other node ID to a load receiver node
            curLoadNodeInfo.SHthisNodeMsg.SHmsgType = SH_MSG_ST_IDLE;
            curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_NOP;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusH = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusL = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusID = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusVal = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHreserved1 = SH_RESERVED_BYTE;
            curLoadNodeInfo.SHthisNodeMsg.SHreserved2 = SH_RESERVED_BYTE;
            curLoadNodeInfo.SHthisNodeMsg.SHchksum = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHcalcChksum = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusTX = 0;
            curLoadNodeInfo.SHthisNodeMsg.SHstatusRX = 0;


        // reset our command repeat counter for the new node
        curSHcmdRepeats = 0;

        // Draw the new load button on wall control LCD screen
        lcdDrawLoadBtn(currentRoomNum, loadNum);     

        // Request current setting and powered state
        SHloadRequestCurrentIntensity(curLoadNodeInfo.SHthisNodeID);
        
        Serial.print("Selecting room ");
        Serial.print(curLoadNodeInfo.SHthisNodeLoc, DEC);
        Serial.print(" / load ");
        Serial.print(loadNum, DEC);
        Serial.print(" as current load, of type ");
        switch(curLoadNodeInfo.SHthisNodeType)
        {
            case NODEINFO_NODETYPE_LIGHT:
                Serial.print("light");
                break;
            case NODEINFO_NDOETYPE_FAN:
                Serial.print("fan");
                break;
            default: 
                break;
        }
        Serial.print(" Load Node ID is ");
        Serial.print(curLoadNodeInfo.SHthisNodeID, HEX);
        Serial.println();
    }
    else
    {
        Serial.print("ERROR - Invalid load number ");
        Serial.println(loadNum, DEC);
    }
}


uint8_t SHloadRequestCurrentIntensity(uint16_t loadNodeID)
{
    curLoadNodeInfo.SHthisNodeID   = loadNodeID;
    curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_READCRNT;

    // TODO 
//    doWCnodeIDmsgSM();
  return(2);
  
    // current level is in the curLoadInfo struct at end of messaging conversation
    return(curLoadNodeInfo.SHthisNodeMsg.SHstatusVal);
}


uint8_t SHloadRequestPoweredState(uint16_t loadNodeID)
{
    curLoadNodeInfo.SHthisNodeID   = loadNodeID;
    curLoadNodeInfo.SHthisNodeMsg.SHcommand = SH_CMD_LOAD_READPWR;

    // TODO 
//    doWCnodeIDmsgSM();
  return(SH_POWERED_ON);
  
    // current level is in the curLoadInfo struct at end of messaging conversation
    return(curLoadNodeInfo.SHthisNodeMsg.SHstatusVal);
}

