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

//#include <ByteSwap.h>


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
uint8_t currentLoadNumInRoom = 0;
uint8_t lastLoadNumInRoom = 0;

// change this to match your SD shield or module;
const int chipSelect = 4; // for Adafruit 2.8" LCD Resistive Touchscreen

//#include "SpiTFTbitmap.h"


//#define SH_WALLCONTROL_LCD_BACKDROP_FILE "/IMAGES/BACKDROP.BMP"
//#define SH_WALLCONTROL_LCD_BACKDROP_FILE "/IMAGES/BD24.BMP"
#define SH_WALLCONTROL_LCD_BACKDROP_FILE "/BCKDRP24.BMP"
#define SH_WALLCONTROL_LCD_BACKDROP_X (uint16_t)240
#define SH_WALLCONTROL_LCD_BACKDROP_Y (uint16_t)320










void setup() {
    // put your setup code here, to run once:
    uint8_t tmpR8 = 0;
    
    Serial.begin(9600);

  
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    #define TFT_ROT_PORTRAIT 0
    tft.setRotation(TFT_ROT_PORTRAIT);

    if (!ts.begin()) { 
        Serial.println("Unable to start touchscreen.");
    } 
    else { 
        Serial.println("Touchscreen started."); 
        ts.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
    }

  
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
        Serial.println("failed!");
    }
    else
    {
        Serial.println("OK!");
        
        currentRoomNum = 0;
        currentLoadNumInRoom = 0;
        //getThisNodeID();
        #define myIDfilename "/ME.BIN"
        if( SD.exists(myIDfilename) )
        {
            uint16_t thisNodeID = 0;
            uint8_t *ptrThisNodeID = (uint8_t *)&thisNodeID;
            File myIDfile = SD.open(myIDfilename, FILE_READ);
            if( myIDfile.available() )
            {
                ptrThisNodeID[1] = (uint8_t)myIDfile.read();
            }
            if( myIDfile.available() )
            {
                ptrThisNodeID[0] = (uint8_t)myIDfile.read();
            }
            myIDfile.close();
            Serial.print("SmartHome Node ID for this control unit = ");
            Serial.println(thisNodeID, HEX);
        }

// /65536/245.BIN$
        char roomNumDirName[7] = "0";
        char loadNumFileName[16] = "/1/2.BIN";

        volatile uint32_t tmpRoomNum = 0;
            volatile uint32_t tmpLoadNum = 0;
        sprintf(roomNumDirName, "/%d", tmpRoomNum);
        while( (tmpRoomNum <= 65535) && SD.exists(roomNumDirName) )
        {
            Serial.print("Checking SD for ");
            Serial.println(roomNumDirName);

            lastRoomNum = tmpRoomNum;
            Serial.print("Found Room # ");
            Serial.println(tmpRoomNum, DEC);
#if 0
            tmpLoadNum = 3;
            sprintf(loadNumFileName, "/%d/%d.BIN", tmpRoomNum, tmpLoadNum);
            while( (tmpLoadNum <= 255) && SD.exists(loadNumFileName) )
            {
                Serial.print("    Checking SD for ");
                Serial.println(loadNumFileName);
                lastLoadNumInRoom = tmpLoadNum;
                
                Serial.print("    Found Load # ");
                Serial.println(tmpLoadNum, DEC);
                char tmpStr[30];
                sprintf(tmpStr, "testing ints %d %d", 3, 7);
                Serial.println(tmpStr);
                
                tmpLoadNum = tmpLoadNum + 1;
//                sprintf(loadNumFileName, "/%d/%d.BIN", tmpRoomNum, ++tmpLoadNum);
                sprintf(loadNumFileName, "/%i/%i.BIN", tmpRoomNum, 7);
                Serial.print("    Load filename for load # ");
                Serial.print(tmpLoadNum, DEC);
                Serial.print(" is ");
                Serial.println(loadNumFileName);
            }
#endif            
            tmpRoomNum += 1;
            sprintf(roomNumDirName, "/%d", tmpRoomNum);
        }        

//        lastRoomNum = 0;
//        lastLoadNumInRoom = 0;

        Serial.print("Current Room = ");
        Serial.print(currentRoomNum, DEC);
        Serial.print(" ; Last Room = ");
        Serial.print(lastRoomNum, DEC);
        Serial.print(" ; Current Load = ");
        Serial.print(currentLoadNumInRoom, DEC);
        Serial.print(" ; Last Load = ");
        Serial.print(lastLoadNumInRoom, DEC);
        Serial.println("");
    }

     
    //bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, SH_WALLCONTROL_LCD_BACKDROP_X, SH_WALLCONTROL_LCD_BACKDROP_Y);
    bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, 0, 0, 240, 320);   
    lcdDrawRoomBtn(0);
    lcdDrawLoadBtn(0, 0);
    lcdDrawLvlIndBar();
    lcdDrawLvlIndicator(5);
    lcdDrawLvlIndicator(4);
    lcdDrawLvlIndicator(3);
    lcdDrawLvlIndicator(2);
    lcdDrawLvlIndicator(1);
    lcdDrawLvlIndicator(0);
    
#if 0
    // Attempt to transmit a TX frame for debugging
//    mySHzigbee.initXmitAPIframe();

    // Fill TX frame payload (SH message) with current message values
    mySHzigbee.prepareTXmsg( (uint16_t)0xabcd,    // SH dest ID
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
    Serial.println("Exiting setup()");
}


#define TS_DEBOUNCE_MILLIS  (uint32_t)400  // number of milliseconds to delay for touchscreen debounce
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

#if 0
        Serial.print("nowMillis=");
        Serial.print(nowMillis, DEC);
        Serial.print(" ; nextMillis=");
        Serial.println(nextMillis, DEC);
#endif

        // TODO Add to this for rollover checking, but careful and test a lot as it's tricky
        if( nowMillis >= nextMillis )
        {
//            Serial.println("Processing the TS event");
            
            prevMillis = nowMillis;
            nextMillis = nowMillis + TS_DEBOUNCE_MILLIS;         
        
            // respond to touchscreen input

            // Scale from ~0->4000 to tft.width using the calibration #'s
            p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
            p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

            // Get x/y coord for our chosen rotation of the image
            x = p.x;
            y = p.y;

#if 1
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
    else
    {
        //NO touchscreen input for this iteration of loop()
    }

}


// Figure out which LCD GUI button was pressed and respond accodringly
void SHdoTouchButton(uint16_t x, uint16_t y)
{
    Serial.print("Touch Button ");  


// FAV 45:55   ; 125:107
// INC 45:10   ; 125:45
// DEC 45:125  ; 125:160
// ON  140:10  ; 230:70
// OFF 140:100 ; 230:160


    // The X and Y coordinates checked here were measured with the intended wall-switch background image displayed, and touching the touchscreen with a stylus
    // and noting the coordinate numbers coming out of the "TouchTest" Example program included with the touch-sensor driver library for this panel.
    if( x>=140 && x<230)
    {
            if(y>10 && y<70)  // ON button
        {
            Serial.println("ON");             

            // Fill TX frame payload (SH message) with current message values
            mySHzigbee.prepareTXmsg( (uint16_t)0xabcd,    // SH dest ID
                                     (uint16_t)0xf00d,    // SH src ID
                                     SH_MSG_TYPE_CMD_REQ, // SH msg Type
                                     SH_CMD_LOAD_ON,      // SH command
                                     (uint8_t)0x00,       // SH statusH
                                     (uint8_t)0x00,       // SH statusL
                                     (uint8_t)0x00        // SH statusVal
                                   );

//            doNodeIDmsgSM(currLoadNodeInfo);
                        
            // DEV remove - send our prepared ZB TX frame out uart to Xbee module for Zigbee transmit
            mySHzigbee.zbXmitAPIframe();
        }
        else if(y>100 && y<160)  // OFF button
        {
            Serial.println("OFF"); 

            // Fill TX frame payload (SH message) with current message values
            mySHzigbee.prepareTXmsg( (uint16_t)0xabcd,    // SH dest ID
                                     (uint16_t)0xf00d,    // SH src ID
                                     SH_MSG_TYPE_CMD_REQ, // SH msg Type
                                     SH_CMD_LOAD_OFF,     // SH command
                                     (uint8_t)0x00,       // SH statusH
                                     (uint8_t)0x00,       // SH statusL
                                     (uint8_t)0x00        // SH statusVal
                                   );

//            doNodeIDmsgSM(currLoadNodeInfo);
                       
            // DEV remove - send our prepared ZB TX frame out uart to Xbee module for Zigbee transmit
            mySHzigbee.zbXmitAPIframe();
        }
        if(y>180 && y<220)  // Room -> button (select next Room)
        {
            Serial.println("Room ->"); 
        }
        else if(y>240 && y<290)  // Load -> button (select next Load)
        {
            Serial.println("Load ->"); 
        }
    }
    else if( (x>=10 && x<100) && (y>180 && y<220) )  // Room <- button (select previous Room)
    {
            Serial.println("Room <-"); 
    }
    else if( (x>=10 && x<100) && (y>240 && y<290) )  // Load <- button (select previous Load)
    {
            Serial.println("Load <-"); 
    }
    else if( x>=45 && x<125) 
    {
        if(y>10 && y<45)  // Up-Arrow button (increase load intensity)
        {
            Serial.println("Increase");             

            // Fill TX frame payload (SH message) with current message values
            mySHzigbee.prepareTXmsg( (uint16_t)0xabcd,    // SH dest ID
                                     (uint16_t)0xf00d,    // SH src ID
                                     SH_MSG_TYPE_CMD_REQ, // SH msg Type
                                     SH_CMD_LOAD_INC,     // SH command
                                     (uint8_t)0x00,       // SH statusH
                                     (uint8_t)0x00,       // SH statusL
                                     (uint8_t)0x00        // SH statusVal
                                   );

//            doNodeIDmsgSM(currLoadNodeInfo);
            
            // DEV remove - send our prepared ZB TX frame out uart to Xbee module for Zigbee transmit
            mySHzigbee.zbXmitAPIframe();
        }
        else if(y>60 && y<110)  // FAV button (favorite intensity value for this load)
        {
            Serial.println("FAV"); 
        }
        else if(y>125 && y<160)  // Down-Arrow button (decrease load intensity)
        {
            Serial.println("Decrease"); 

            // Fill TX frame payload (SH message) with current message values
            mySHzigbee.prepareTXmsg( (uint16_t)0xabcd,    // SH dest ID
                                     (uint16_t)0xf00d,    // SH src ID
                                     SH_MSG_TYPE_CMD_REQ, // SH msg Type
                                     SH_CMD_LOAD_DEC,     // SH command
                                     (uint8_t)0x00,       // SH statusH
                                     (uint8_t)0x00,       // SH statusL
                                     (uint8_t)0x00        // SH statusVal
                                   );

//            doNodeIDmsgSM(currLoadNodeInfo);
            
            // DEV remove - send our prepared ZB TX frame out uart to Xbee module for Zigbee transmit
            mySHzigbee.zbXmitAPIframe();
        }
    }    
    else
    {
        printInvalidCoord(x, y);
    }

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


void lcdDrawLvlIndBar(void)
{
    #define FILENAME_LVL_IND_BAR "/INDBAR.BMP"
    Serial.print("Drawing Indicator Bar");
    Serial.print(" using filename ");
    Serial.println(FILENAME_LVL_IND_BAR);
//    bmpDraw(FILENAME_LVL_IND_BAR, 118, 12, 240, 320);
    bmpDraw(FILENAME_LVL_IND_BAR, 0, 12, 240, 320);
}

void lcdDrawLvlIndicator(uint8_t myLevel)
{
    // Redraw the Level indicator bar to clear out the current/previous level drawing position
    //lcdDrawLvlIndBar();
    
    #define FILENAME_LVL_IND "/LVLIND.BMP"
    Serial.print("Drawing Indicator Bar");
    Serial.print(" using filename ");
    Serial.println(FILENAME_LVL_IND);

    switch(myLevel)
    {
        case (uint8_t)5: // Full ON
            bmpDraw(FILENAME_LVL_IND, 0, 17, 240, 320); //15
            break;

        case (uint8_t)4: 
            bmpDraw(FILENAME_LVL_IND, 0, 44, 240, 320); // 15+18
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
            bmpDraw(FILENAME_LVL_IND, 0, 150, 240, 320); // 168 - height of level indicator (29)
            break;

        default:
            // Do nothing
            break;
    }        
}


void lcdDrawRoomBtn(uint16_t roomNum)
{
    char btnFileName[9];

    sprintf(btnFileName, "/%d/R.BMP", roomNum);

    Serial.print("Drawing Button graphic for room ");
    Serial.print(roomNum, DEC);
    Serial.print(" using filename ");
    Serial.println(btnFileName);
    
    bmpDraw(btnFileName, 0, 191, 240, 320);
}

// /65536/255.bmp$
void lcdDrawLoadBtn(uint16_t roomNum, uint8_t loadNum)
{
    char btnFileName[16];

    sprintf(btnFileName, "/%d/%d.BMP", roomNum, loadNum);

    Serial.print("Drawing Button graphic for room ");
    Serial.print(roomNum, DEC);
    Serial.print(" using filename ");
    Serial.println(btnFileName);
    
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

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

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
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
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


