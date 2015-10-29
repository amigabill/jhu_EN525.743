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
//#include <SmartHome_Zigbee.h>

#include <ByteSwap.h>


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


// change this to match your SD shield or module;
const int chipSelect = 4; // for Adafruit 2.8" LCD Resistive Touchscreen

//#include "SpiTFTbitmap.h"


//#define SH_WALLCONTROL_LCD_BACKDROP_FILE "/IMAGES/BACKDROP.BMP"
#define SH_WALLCONTROL_LCD_BACKDROP_FILE "/IMAGES/BD24.BMP"
#define SH_WALLCONTROL_LCD_BACKDROP_X (uint16_t)240
#define SH_WALLCONTROL_LCD_BACKDROP_Y (uint16_t)320




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
    volatile uint8_t  SHstatusH;    // High Byte of 16bit Smarthome
    volatile uint8_t  SHstatusL;    // Low byte of 16bbit Smarthome
    volatile uint8_t  SHstatusVal;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved1;  // 8bit Smarthome message type
    volatile uint8_t  SHreserved2;  // 8bit Smarthome message type
    volatile uint8_t  SHpayldCRC;   // 8bit Smarthome message type
} SHpayload, *prtSHpayload;


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

ZBframeTX     myZBframeTX;
////prtZBframeTX  ptrMyZBframeTX = &myZBframeTX;

#define ZB_IN_FRAME_YES 1
#define ZB_IN_FRAME_NO  0







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
#if 0
        // 20151026 wtoner - changed form ZYX mode to only XY mode.
        //writeRegister8(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
        tmpR8 = ts.readRegister8(STMPE_TSC_CTRL); // XY and enable 
        tmpR8 &= ~STMPE_TSC_CTRL_EN;
        ts.writeRegister8(STMPE_TSC_CTRL, tmpR8); // XY and enable 
        ts.writeRegister8(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XY | STMPE_TSC_CTRL_EN); // XY and enable 
#endif

        // clear the FIFO data
//        ts.writeRegister8(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
//        ts.writeRegister8(STMPE_FIFO_STA, 0x00); // reset all ints

        ts.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
        //ts.getPoint(); // clear out bogus initial touch event
    }
  
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
        Serial.println("failed!");
    }
    Serial.println("OK!");
      
    //bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, SH_WALLCONTROL_LCD_BACKDROP_X, SH_WALLCONTROL_LCD_BACKDROP_Y);
    bmpDraw(SH_WALLCONTROL_LCD_BACKDROP_FILE, 0, 0, 240, 320);   

#if 0
    // Attempt to transmit a TX frame for debugging
    initXmitAPIframe();

#if 0
    void prepareTXmsg( uint16_t prepSHsrcID,      // Source ID
                   uint16_t prepSHdestID,     // Dest ID
                   uint8_t  prepSHmsgType,    // Msg Type
                   uint8_t  prepSHcommand,    // CMD
                   uint8_t  prepSHstatusH,
                   uint8_t  prepSHstatusL,
                   uint8_t  prepSHstatusVal
                 )
#endif

    prepareTXmsg( (uint16_t)0xabcd,
                  (uint16_t)0xf00d,
                  SH_MSG_TYPE_CMD_REQ,
                  SH_CMD_LOAD_ON,
                  (uint8_t)0x00,
                  (uint8_t)0x00,
                  (uint8_t)0x00
                );
#endif
    zbXmitAPIframe();

    Serial.println("");
    Serial.println("");
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

#if 0
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

    // The X and Y coordinates checked here were measured with the intended wall-switch background image displayed, and touching the touchscreen with a stylus
    // and noting the coordinate numbers coming out of the "TouchTest" Example program included with the touch-sensor driver library for this panel.
    if( x>=20 && x<100)
    {
        if(y>10 && y<70)
        {
            Serial.print("ON");             
        }
        else if(y>100 && y<160)
        {
            Serial.print("OFF"); 
        }
        else if(y>180 && y<220)
        {
            Serial.print("Room <-"); 
        }
        else if(y>240 && y<290)
        {
            Serial.print("Load <-"); 
        }
        else
        {
            printInvalidCood(x, y);
        }
    }
    else if( x>=150 && x<230)
    {
        if(y>10 && y<45)
        {
            Serial.print("Increase");             
        }
        else if(y>60 && y<110)
        {
            Serial.print("FAV"); 
        }
        else if(y>125 && y<160)
        {
            Serial.print("Decrease"); 
        }
        else if(y>180 && y<220)
        {
            Serial.print("Room ->"); 
        }
        else if(y>240 && y<290)
        {
            Serial.print("Load ->"); 
        }
        else
        {
            printInvalidCood(x, y);
        }
    }
    else
    {
        printInvalidCood(x, y);
    }

    Serial.println("");
}

void printInvalidCood(uint16_t x, uint16_t y)
{
    Serial.print("INVALID coordinate (no button at) ");
    Serial.print(x, DEC);
    Serial.print(" : ");
    Serial.print(y, DEC);
}


// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

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


void initXmitAPIframe(void)
{
    myZBframeTX.ZBfrmDelimiter = ZB_START_DELIMITER;
    myZBframeTX.ZBfrmLength = BYTESWAP16( (uint16_t)26 );
    myZBframeTX.ZBfrmType= ZB_FRAME_TYPE_TX_REQ;
    myZBframeTX.ZBfrmID = (uint8_t)1;
    myZBframeTX.ZBdaddr64High = BYTESWAP32( ZB_64ADDR_BCAST_HIGH );
    myZBframeTX.ZBdaddr64Low = BYTESWAP32( ZB_64ADDR_BCAST_LOW );
    myZBframeTX.ZBdaddr16 = BYTESWAP16( ZB_16ADDR_BCAST );
    myZBframeTX.ZBfrmRadius = ZB_BCAST_RADIUS;
    myZBframeTX.ZBfrmOptions = ZB_OPTIONS;
}

// Transmit a Zigbee API TX Request Frame
// Each Zigbee message frame has a 12byte payload, the SmartHome Message,
// so length=23=0x17 bytes, total frame=27bytes
uint8_t zbXmitAPIframe(void)
{
    uint8_t txBuffer[30];
    txBuffer[0] = (uint8_t)0x7e;
    txBuffer[1] = (uint8_t)0x00;
    txBuffer[2] = (uint8_t)0x1a;
    txBuffer[3] = (uint8_t)0x10;
    txBuffer[4] = (uint8_t)0x01;
    txBuffer[5] = (uint8_t)0x00;
    txBuffer[6] = (uint8_t)0x00;
    txBuffer[7] = (uint8_t)0x00;
    txBuffer[8] = (uint8_t)0x00;
    txBuffer[9] = (uint8_t)0x00;
    txBuffer[10] = (uint8_t)0x00;
    txBuffer[11] = (uint8_t)0xff;
    txBuffer[12] = (uint8_t)0xff;
    txBuffer[13] = (uint8_t)0xff;
    txBuffer[14] = (uint8_t)0xfe;
    txBuffer[15] = (uint8_t)0x00;
    txBuffer[16] = (uint8_t)0x00;
    txBuffer[17] = (uint8_t)0xab;
    txBuffer[18] = (uint8_t)0xcd;
    txBuffer[19] = (uint8_t)0xf0;
    txBuffer[20] = (uint8_t)0x0d;
    txBuffer[21] = (uint8_t)0x01;
    txBuffer[22] = (uint8_t)0x01;
    txBuffer[23] = (uint8_t)0x00;
    txBuffer[24] = (uint8_t)0x00;
    txBuffer[25] = (uint8_t)0x00;
    txBuffer[26] = (uint8_t)0x54;
    txBuffer[27] = (uint8_t)0x58;
    txBuffer[28] = (uint8_t)0xdc;
    txBuffer[29] = (uint8_t)0xf4;

    Serial.write(txBuffer, 30);

    return( (uint8_t)0x00 );
    
    // First populate the TX frame payload data (aka RF Data)

    // initialize the Zigbee TX API frame checksum before calculating it
    myZBframeTX.ZBfrmChksum = 0;

    myZBframeTX.ZBfrmPayload.SHpayldCRC = 0;

    Serial.println("Starting a TX API frame now:");
    
    // send each TX frame field in turn, and update checksum on relevant fields
//    Serial.write( myZBframeTX.ZBfrmDelimiter );
    Serial.print(myZBframeTX.ZBfrmDelimiter, HEX);
    Serial.print(" ");
    DEBUGprintTXfrmChkSum();

//    Serial.write( BYTESWAP16(myZBframeTX.ZBfrmLength) );
    Serial.print( BYTESWAP16(myZBframeTX.ZBfrmLength), HEX);
    Serial.print(" ");
    DEBUGprintTXfrmChkSum();


    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmType);
//    Serial.write( myZBframeTX.ZBfrmType );
//    Serial.print(myZBframeTX.ZBfrmType, HEX);
//    Serial.print(" ");
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmID);
//    Serial.write( myZBframeTX.ZBfrmID );
//    Serial.print(myZBframeTX.ZBfrmID, HEX);
//    Serial.print(" ");
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum32(myZBframeTX.ZBdaddr64High);
//    Serial.write( BYTESWAP32(myZBframeTX.ZBdaddr64High) );
//    Serial.print( BYTESWAP32(myZBframeTX.ZBdaddr64High), HEX);
//    Serial.print(" ");
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum32(myZBframeTX.ZBdaddr64Low);
//    Serial.write( BYTESWAP32(myZBframeTX.ZBdaddr64Low) );
//    Serial.print( BYTESWAP32(myZBframeTX.ZBdaddr64Low), HEX);
//    Serial.print(" ");
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum16(myZBframeTX.ZBdaddr16);
//    Serial.write( BYTESWAP16(myZBframeTX.ZBdaddr16) );
//    Serial.print( BYTESWAP16(myZBframeTX.ZBdaddr16), HEX);
//    Serial.print(" ");
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmRadius);
//    Serial.write( myZBframeTX.ZBfrmRadius );
//    Serial.print(myZBframeTX.ZBfrmRadius, HEX);
//    Serial.print(" ");
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmOptions);
//    Serial.write( myZBframeTX.ZBfrmOptions );
//    Serial.print(myZBframeTX.ZBfrmOptions, HEX);
//    Serial.print(" ");
    DEBUGprintTXfrmChkSum();


    myZBframeTX.ZBfrmChksum += calcChkSum16(myZBframeTX.ZBfrmPayload.SHsrcID);
//    Serial.write( BYTESWAP16(myZBframeTX.ZBfrmPayload.SHsrcID) );
//    Serial.print( BYTESWAP16(myZBframeTX.ZBfrmPayload.SHsrcID), HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum16(myZBframeTX.ZBfrmPayload.SHsrcID);
    DEBUGprintTXfrmChkSum();

    // Do the smartHome Message payload portion of the Zigbee TX API Frame
    myZBframeTX.ZBfrmChksum += calcChkSum16(myZBframeTX.ZBfrmPayload.SHdestID);
//    Serial.write( BYTESWAP16(myZBframeTX.ZBfrmPayload.SHdestID) );
//    Serial.print( BYTESWAP16(myZBframeTX.ZBfrmPayload.SHdestID), HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum16(myZBframeTX.ZBfrmPayload.SHdestID);
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHmsgType);
//    Serial.write( myZBframeTX.ZBfrmPayload.SHmsgType );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHmsgType, HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum8(myZBframeTX.ZBfrmPayload.SHmsgType);
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHcommand);
//    Serial.write( myZBframeTX.ZBfrmPayload.SHcommand );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHcommand, HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum8(myZBframeTX.ZBfrmPayload.SHcommand);
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusH);
//    Serial.write( myZBframeTX.ZBfrmPayload.SHstatusH );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHstatusH, HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusH);
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusL);
//    Serial.write( myZBframeTX.ZBfrmPayload.SHstatusL );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHstatusL, HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusL);
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusVal);
//    Serial.write( myZBframeTX.ZBfrmPayload.SHstatusVal );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHstatusVal, HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusVal);
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved1);
//    Serial.write( myZBframeTX.ZBfrmPayload.SHreserved1 );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHreserved1, HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved1);
    DEBUGprintTXfrmChkSum();

    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved2);
//    Serial.write( myZBframeTX.ZBfrmPayload.SHreserved2 );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHreserved2, HEX);
//    Serial.print(" ");
////    myZBframeTX.ZBfrmPayload.SHpayldCRC += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved2);
    DEBUGprintTXfrmChkSum();

    // Calculate the SmartHome Message payload's internal checksum
////    myZBframeTX.ZBfrmPayload.SHpayldCRC = (uint8_t)0xff - myZBframeTX.ZBfrmPayload.SHpayldCRC;
//    Serial.write( myZBframeTX.ZBfrmPayload.SHpayldCRC );
//    Serial.print(myZBframeTX.ZBfrmPayload.SHpayldCRC, HEX);
//    Serial.print(" ");
//    myZBframeTX.ZBfrmChksum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHpayldCRC);
//    Serial.print(myZBframeTX.ZBfrmPayload.SHpayldCRC, HEX);
    DEBUGprintTXfrmChkSum();


    // Save/send the final Zigbee TX API Frame checksum
    myZBframeTX.ZBfrmChksum = (uint8_t)0xff - myZBframeTX.ZBfrmChksum;
//    Serial.write( myZBframeTX.ZBfrmChksum );
    Serial.print(myZBframeTX.ZBfrmChksum, HEX);

    Serial.println("");
    Serial.print("Finished sending TXframe");
}


uint8_t  calcChkSum8(uint8_t ui8)
{
    Serial.print(ui8, HEX);
    Serial.print(" ");
    return(ui8);
}

uint8_t  calcChkSum16(uint16_t ui16)
{
    uint8_t *ptrUI8AsUi16 = (uint8_t *)&ui16;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi16[0];
    Serial.print(ptrUI8AsUi16[0], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi16[1];
    Serial.print(ptrUI8AsUi16[1], HEX);
    Serial.print(" ");

    return(tmpChkSum);
}

uint8_t  calcChkSum32(uint32_t ui32)
{
    uint8_t *ptrUI8AsUi32 = (uint8_t *)&ui32;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi32[0];
    Serial.print(ptrUI8AsUi32[0], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[1];
    Serial.print(ptrUI8AsUi32[1], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[2];
    Serial.print(ptrUI8AsUi32[2], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[3];
    Serial.print(ptrUI8AsUi32[3], HEX);
    Serial.print(" ");

    return(tmpChkSum);
}

//void DEBUGprintTXfrmChkSum(uint8_t chkSum)
void DEBUGprintTXfrmChkSum(void)
{
    #if 1
    Serial.print("<cs=");
    //Serial.println(chkSum);
    Serial.print(myZBframeTX.ZBfrmChksum, HEX);
    Serial.print("> ");
    #endif
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
    uint8_t tmpChkSum = 0;
    
    myZBframeTX.ZBfrmPayload.SHpayldCRC= 0;
    
    // ints are 16bit Little Endian, longs are 32bit Little Endian
    // Zigbee goes Big Endian
    myZBframeTX.ZBfrmPayload.SHdestID    = BYTESWAP16(prepSHdestID);
    tmpChkSum += calcChkSum16(myZBframeTX.ZBfrmPayload.SHdestID);
    
    myZBframeTX.ZBfrmPayload.SHsrcID     = BYTESWAP16(prepSHsrcID);
    tmpChkSum += calcChkSum16(myZBframeTX.ZBfrmPayload.SHsrcID);

    myZBframeTX.ZBfrmPayload.SHmsgType   = prepSHmsgType;
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHmsgType);

    myZBframeTX.ZBfrmPayload.SHcommand   = prepSHcommand;
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHcommand);

    myZBframeTX.ZBfrmPayload.SHstatusH   = prepSHstatusH;
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHdestID);

    myZBframeTX.ZBfrmPayload.SHstatusL   = prepSHstatusL;
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusL);

    myZBframeTX.ZBfrmPayload.SHstatusVal = prepSHstatusVal;
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusVal);

    myZBframeTX.ZBfrmPayload.SHreserved1 = (uint8_t)'T';
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved1);

    myZBframeTX.ZBfrmPayload.SHreserved2 = (uint8_t)'X';
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved2);

    //calc payload message checksum
//    myZBframeTX.ZBfrmPayload.SHpayldCRC  = calcSHmsgChkSumTX();
    tmpChkSum = (uint8_t)0xff - tmpChkSum;
    myZBframeTX.ZBfrmPayload.SHpayldCRC  = tmpChkSum;
    Serial.print("<mcs=");
    Serial.print(myZBframeTX.ZBfrmPayload.SHpayldCRC, HEX);
    Serial.print("> ");
}


// not quite right due to 16bit ints in there
// Calculate the SmartHome Message payload's internal checksum
uint8_t calcSHmsgChkSumTX(void)
{
    uint8_t tmpChkSum = 0;

    tmpChkSum += calcChkSum16(myZBframeTX.ZBfrmPayload.SHdestID);
    tmpChkSum += calcChkSum16(myZBframeTX.ZBfrmPayload.SHsrcID);
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHmsgType);
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHcommand);
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusH);
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusL);
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHstatusVal);
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved1);
    tmpChkSum += calcChkSum8(myZBframeTX.ZBfrmPayload.SHreserved2);
    tmpChkSum = (uint8_t)0xff - tmpChkSum;

    return(tmpChkSum);
}
