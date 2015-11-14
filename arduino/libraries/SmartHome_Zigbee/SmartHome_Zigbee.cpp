// Include Arduino standard libraries
#include "Arduino.h"

#include "../ByteSwap/ByteSwap.h"

#include "SmartHome_Zigbee.h"


// Comment out the following defines to save Arduino resources
// UNcomment them to activate the related debug output
//#define DEBUG_ZB_XMIT
//#define DEBUG_ZB_RECEIVE


// constructor
SHzigbee::SHzigbee()
{
	_initXmitAPIframe();
        //initSHmsgRX();
	
        _ZBoffsetRXbuff = ZB_FRM_OFFSET_DELMTR; //ZB_START_DELIMITER; //Delimiter byte is offset 0 into RX buffer	

	ZBnewFrameRXed = NO;
	newSHmsgRX = NO;
	ZBinFrameRX = NO;
}



// Initialize the TX API frame with values we will use as standard
void SHzigbee::_initXmitAPIframe(void)
{
    _myZBframeTX.ZBfrmDelimiter  = ZB_START_DELIMITER; // 0x7e;
    _myZBframeTX.ZBfrmLength     = ZB_TX_FRM_LEN; 
    _myZBframeTX.ZBfrmType       = ZB_FRAME_TYPE_TX_REQ;
    _myZBframeTX.ZBfrmID         = (uint8_t)1; // always use Zigbee Frame ID of 1
    _myZBframeTX.ZBdaddr64High   = ZB_64ADDR_BCAST_HIGH;
    _myZBframeTX.ZBdaddr64Low    = ZB_64ADDR_BCAST_LOW;
    _myZBframeTX.ZBdaddr16       = ZB_16ADDR_BCAST;
    _myZBframeTX.ZBfrmRadius     = ZB_BCAST_RADIUS;
    _myZBframeTX.ZBfrmOptions    = ZB_OPTIONS;
}


// Transmit a Zigbee API TX Request Frame
// Each Zigbee message frame has a 12byte payload, the SmartHome Message,
// so length=23=0x17 bytes, total frame=27bytes (<- double check that)
// Zigbee data is defined to be Big-Endian (BE) while Arduino AVR is Little-Endian (LE)
// so will need to take care to assemble 16bit and 32bit ints correctly
uint8_t SHzigbee::zbXmitAPIframe(void)
{
    // use some pointer to unsigned int 8 (byte) to access parts of larger datatypes a byte at a time during Zigbee frame transmit
    uint8_t *ptrZBfrmLength           = (uint8_t *)&(_myZBframeTX.ZBfrmLength);
    uint8_t *ptrZBdaddr64H            = (uint8_t *)&(_myZBframeTX.ZBdaddr64High);
    uint8_t *ptrZBdaddr64L            = (uint8_t *)&(_myZBframeTX.ZBdaddr64Low);
    uint8_t *ptrZBfrmPldSHdestAddr16  = (uint8_t *)&(_myZBframeTX.ZBfrmPayload.SHdestID);
    uint8_t *ptrZBfrmPldSHsrcAddr16   = (uint8_t *)&(_myZBframeTX.ZBfrmPayload.SHsrcID);
    uint8_t *ptrSHdestAddr16          = (uint8_t *)&(SHmsgTX.SHdestID); // Pointer into SHmsgRX received SmartHome message struct
    uint8_t *ptrSHsrcAddr16           = (uint8_t *)&(SHmsgTX.SHsrcID);  // Pointer into SHmsgRX received SmartHome message struct


    // initialize the Zigbee TX API frame checksum before calculating it
    _myZBframeTX.ZBfrmChksum = 0;

    // Get data our of our Zigbee frame struct and into a buffer array of bytes/uint8_t 
    // for Serial.write() call to Zbee module
#if 0
    Serial.print("In zbXmitAPIframe ; ZB_TX_FRM_BYTES=");
    Serial.print(ZB_TX_FRM_BYTES, DEC);
    Serial.print(" ; ZB_TX_FRM_LEN=");
    Serial.print(ZB_TX_FRM_LEN, DEC);
    Serial.println("");
#endif

    Serial.println("Starting a TX API frame now:");

    _ZBfrmBufferTX[ZB_FRM_OFFSET_DELMTR]       = _myZBframeTX.ZBfrmDelimiter; //(uint8_t)0x7e;

    _ZBfrmBufferTX[ZB_FRM_OFFSET_LENH]         = ptrZBfrmLength[1]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_LENL]         = ptrZBfrmLength[0]; //(uint8_t)0x1a;

    // START calculating ZB frame checksum, also the ZB "Frame Length" begins here
    
    _ZBfrmBufferTX[ZB_FRM_OFFSET_FTYPE]        = _myZBframeTX.ZBfrmType; //(uint8_t)0x10;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmType);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_FID]          = _myZBframeTX.ZBfrmID; //(uint8_t)0x01;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmID);

    // 64bit Zigbee addr
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B7] = ptrZBdaddr64H[3]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B6] = ptrZBdaddr64H[2]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B5] = ptrZBdaddr64H[1]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B4] = ptrZBdaddr64H[0]; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += _calcChkSum32(_myZBframeTX.ZBdaddr64High);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B3] = ptrZBdaddr64L[3]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B2] = ptrZBdaddr64L[2]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B1] = ptrZBdaddr64L[1]; //(uint8_t)0xff;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B0] = ptrZBdaddr64L[0]; //(uint8_t)0xff;
    _myZBframeTX.ZBfrmChksum += _calcChkSum32(_myZBframeTX.ZBdaddr64Low);

    // 16bit Zigbee addr
    uint8_t *ptrZBdaddr16 = (uint8_t *)&(_myZBframeTX.ZBdaddr16);
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR16H] = ptrZBdaddr16[1]; //(uint8_t)0xff;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR16L] = ptrZBdaddr16[0]; //(uint8_t)0xfe;
    _myZBframeTX.ZBfrmChksum += _calcChkSum16(_myZBframeTX.ZBdaddr16);

    // Zigbee Radius and Options bytes
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_BRADIUS] = _myZBframeTX.ZBfrmRadius; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmRadius);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_OPTIONS] = _myZBframeTX.ZBfrmOptions; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmOptions);


    // SmartHome message payload bytes
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H]  = ptrZBfrmPldSHdestAddr16[1]; //(uint8_t)0xab;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L]  = ptrZBfrmPldSHdestAddr16[0]; //(uint8_t)0xcd;
    _myZBframeTX.ZBfrmChksum += _calcChkSum16(_myZBframeTX.ZBfrmPayload.SHdestID);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H]   = ptrZBfrmPldSHsrcAddr16[1]; //(uint8_t)0xf0;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L]   = ptrZBfrmPldSHsrcAddr16[0]; //(uint8_t)0x0d;
    _myZBframeTX.ZBfrmChksum += _calcChkSum16(_myZBframeTX.ZBfrmPayload.SHsrcID);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE]   = _myZBframeTX.ZBfrmPayload.SHmsgType; //(uint8_t)0x01;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHmsgType);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_CMD]        = _myZBframeTX.ZBfrmPayload.SHcommand; //(uint8_t)0x01;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHcommand);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_H]   = _myZBframeTX.ZBfrmPayload.SHstatusH; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusH);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_L]   = _myZBframeTX.ZBfrmPayload.SHstatusL; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusL);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL] = _myZBframeTX.ZBfrmPayload.SHstatusVal; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusVal);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_RESVD_1]    = _myZBframeTX.ZBfrmPayload.SHreserved1; //(uint8_t)0x54;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved1);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_RESVD_2]    = _myZBframeTX.ZBfrmPayload.SHreserved2; //(uint8_t)0x58;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved2);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_CHKSUM]     = _myZBframeTX.ZBfrmPayload.SHpayldChksum; //(uint8_t)0xdc;
    _myZBframeTX.ZBfrmChksum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHpayldChksum);


    // STOP calculating ZB frame checksum, also ZB "Frame Length" ends here
    _myZBframeTX.ZBfrmChksum= (uint8_t)0xff -  _myZBframeTX.ZBfrmChksum;
     
    // Zigbee Frame Checksum
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_CHKSUM] = _myZBframeTX.ZBfrmChksum; //(uint8_t)0xf4; //_myZBframeTX.ZBfrmChksum;


#if 1
    Serial.println("");
    uint8_t i=0;
    for(i-0; i<ZB_TX_FRM_BYTES; i++)
    {
        Serial.print( _ZBfrmBufferTX[i], HEX );
        Serial.print(" ");
    }
    Serial.println("");
#endif    

//    Serial.println("");

    Serial.write(_ZBfrmBufferTX, ZB_TX_FRM_BYTES);
    Serial.println("Finished sending TXframe");
}


uint8_t SHzigbee::_calcChkSum8(uint8_t ui8)
{
//    Serial.print(ui8, HEX);
//    Serial.print(" ");
    return(ui8);
}

uint8_t SHzigbee::_calcChkSum16(uint16_t ui16)
{
    uint8_t *ptrUI8AsUi16 = (uint8_t *)&ui16;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi16[1];
//    Serial.print(ptrUI8AsUi16[1], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi16[0];
//    Serial.print(ptrUI8AsUi16[0], HEX);
//    Serial.print(" ");

    return(tmpChkSum);
}

uint8_t SHzigbee::_calcChkSum32(uint32_t ui32)
{
    uint8_t *ptrUI8AsUi32 = (uint8_t *)&ui32;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi32[3];
//    Serial.print(ptrUI8AsUi32[3], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[2];
//    Serial.print(ptrUI8AsUi32[2], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[1];
//    Serial.print(ptrUI8AsUi32[1], HEX);
//    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[0];
//    Serial.print(ptrUI8AsUi32[0], HEX);
//    Serial.print(" ");

    return(tmpChkSum);
}


#if 0
//void DEBUGprintTXfrmChkSum(uint8_t chkSum)
void SHzigbee::DEBUGprintTXfrmChkSum(void)
{
    #if 1
    Serial.print("<cs=");
    //Serial.println(chkSum);
    Serial.print(_myZBframeTX.ZBfrmChksum, HEX);
    Serial.print("> ");
    #endif
}
#endif


// Prepare the TX frame message payload to be sent
void SHzigbee::prepareTXmsg( uint16_t prepSHdestID,     // Dest ID
                             uint16_t prepSHsrcID,      // Source ID
                             uint8_t  prepSHmsgType,    // Msg Type
                             uint8_t  prepSHcommand,    // CMD
                             uint8_t  prepSHstatusH,    // Status/StatusID High byte
                             uint8_t  prepSHstatusL,    // Status/StatusID Low byte
                             uint8_t  prepSHstatusVal   // Status value (8bit)
                           )
{
    uint8_t tmpChkSum = 0;
    
    _myZBframeTX.ZBfrmPayload.SHpayldChksum= 0;
    
    // ints are 16bit Little Endian, longs are 32bit Little Endian
    // Zigbee goes Big Endian
    _myZBframeTX.ZBfrmPayload.SHdestID    = prepSHdestID; //BYTESWAP16(prepSHdestID);
    tmpChkSum += _calcChkSum16(_myZBframeTX.ZBfrmPayload.SHdestID);
    
    _myZBframeTX.ZBfrmPayload.SHsrcID     = prepSHsrcID; //BYTESWAP16(prepSHsrcID);
    tmpChkSum += _calcChkSum16(_myZBframeTX.ZBfrmPayload.SHsrcID);

    _myZBframeTX.ZBfrmPayload.SHmsgType   = prepSHmsgType;
    tmpChkSum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHmsgType);

    _myZBframeTX.ZBfrmPayload.SHcommand   = prepSHcommand;
    tmpChkSum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHcommand);

    _myZBframeTX.ZBfrmPayload.SHstatusH   = prepSHstatusH;
    tmpChkSum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusH);

    _myZBframeTX.ZBfrmPayload.SHstatusL   = prepSHstatusL;
    tmpChkSum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusL);

    _myZBframeTX.ZBfrmPayload.SHstatusVal = prepSHstatusVal;
    tmpChkSum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusVal);

    _myZBframeTX.ZBfrmPayload.SHreserved1 = (uint8_t)'T';
    tmpChkSum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved1);

    _myZBframeTX.ZBfrmPayload.SHreserved2 = (uint8_t)'X';
    tmpChkSum += _calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved2);

    // finish calc payload message checksum
    tmpChkSum = (uint8_t)0xff - tmpChkSum;
    _myZBframeTX.ZBfrmPayload.SHpayldChksum  = tmpChkSum;

    #if 0
    Serial.print("<mcs=");
    Serial.print(_myZBframeTX.ZBfrmPayload.SHpayldChksum, HEX);
    Serial.print("> ");
    #endif
}


uint8_t SHzigbee::getMsgTypeTX(void)
{
    return( _myZBframeTX.ZBfrmPayload.SHmsgType);
}





// Receive (attempt to) a Zigbee API RX Received Frame
// Each Zigbee RX RCVD message frame has a 12byte payload, the SmartHome Message,
// so length=24=0x18 bytes, total frame=28bytes (<- double check that)
// Zigbee data is defined to be Big-Endian (BE) while Arduino AVR is Little-Endian (LE)
// so will need to take care to assemble 16bit and 32bit ints correctly
//
// NOTE: Prevoius attempts to receive wre flaky and dropped characters expected inthe message, spradically.
// Now, this works better, to get the received bytes into a buffer ASAP, then later go through the buffer
// and see what we get there for the other data structures to store.
// The previous attempt was to try and get each byte into data structures, update running checksum, etc. per byte received,
// and now that seems to maybe have been too much to do between bytes during the serial transfer, as I'm guessing that
// the Zbee RX buffer became full and bytes were tossed out if nto enough room for them, if this was not reading them fast enough.
uint8_t SHzigbee::zbRcvAPIframe(void)
{
    uint8_t ZB_frm_byte = 0; // byte received in uart RX by Serial.read

    uint16_t ZBfrmLen16bit = 0;
    uint8_t  *ptrFrmLen16bit = (uint8_t *)&ZBfrmLen16bit;
    uint8_t *ptrZBbufFrmType = _ZBfrmBufferRX + ZB_FRM_OFFSET_FTYPE;


    // Check if a new ZB frame is incoming
    if(Serial.available() > 0)
    {
	//Serial.println("HAVE serial data");

        // Read a byte from uart RX buffer
        ZB_frm_byte = Serial.read();

        // check if starting a new frame
        if ((NO == ZBinFrameRX) && (ZB_START_DELIMITER == ZB_frm_byte))
	{
            // beginning a new frame
            ZBinFrameRX = YES;
            _ZBoffsetRXbuff = 0; //ZB_START_DELIMITER; //Delimiter byte is offset 0 into RX buffer

            _ZBfrmBufferRX[ZB_FRM_OFFSET_DELMTR] = ZB_frm_byte;

            while (Serial.available() == 0);  // wait for Zigbee Frame length BE MSByte
            //ZB_frm_byte = Serial.read();
            _ZBfrmBufferRX[ZB_FRM_OFFSET_LENH] = Serial.read();

            while (Serial.available() == 0);  // wait for Zigbee Frame length BE LSbyte
            //ZB_frm_byte = Serial.read();
            _ZBfrmBufferRX[ZB_FRM_OFFSET_LENL] = Serial.read();


            ptrFrmLen16bit[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENH];
            ptrFrmLen16bit[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENL];

	    // check Z frame length and ignore frames not matching our SmartHome message frame length
	    if(ZB_RX_FRM_LEN == ZBfrmLen16bit)
	    {
	        // read remainder of frame into buffer, including the checksum which is not included in frame length size
	        Serial.readBytes( ptrZBbufFrmType, (ZBfrmLen16bit+1) );

                #ifdef DEBUG_ZB_RECEIVE
	            // debug put frame buffer content to serial monitor for viewing
	            _debugPrintZBframeBufRX();
                #endif // DEBUG_ZB_RECEIVE

	        // pull data fields out of RX buffer into _myZBframeRX and SHmsgRX data structures
	        _parseZBbufferRX();

		// indicate to other code that a new SmartHome message has been received for processing
		newSHmsgRX = YES;
	    }
	    
	    // indicate not still receiving a ZB frame
            ZBinFrameRX = NO;
	}
    }
}


// Copy the various data fields from the RX Zigbee frame buffer for use
void SHzigbee::_parseZBbufferRX(void)
{
    // use some pointer to unsigned int 8 (byte) to access parts of larger datatypes a byte at a time during Zigbee frame receive
    uint8_t *ptrZBfrmLength           = (uint8_t *)&(_myZBframeRX.ZBfrmLength);
    uint8_t *ptrZBsaddr64H            = (uint8_t *)&(_myZBframeRX.ZBsaddr64High);
    uint8_t *ptrZBsaddr64L            = (uint8_t *)&(_myZBframeRX.ZBsaddr64Low);
    uint8_t *ptrZBsaddr16             = (uint8_t *)&(_myZBframeRX.ZBsaddr16);
    uint8_t *ptrZBfrmPldSHdestAddr16  = (uint8_t *)&(_myZBframeRX.ZBfrmPayload.SHdestID);
    uint8_t *ptrZBfrmPldSHsrcAddr16   = (uint8_t *)&(_myZBframeRX.ZBfrmPayload.SHsrcID);
    uint8_t *ptrSHdestAddr16          = (uint8_t *)&(SHmsgRX.SHdestID); // Pointer into SHmsgRX received SmartHome message struct
    uint8_t *ptrSHsrcAddr16           = (uint8_t *)&(SHmsgRX.SHsrcID);  // Pointer into SHmsgRX received SmartHome message struct
//    uint8_t ZBchksumFromSender = 0; // checksum sent to us for comparison

    _myZBframeRX.ZBfrmChksum = 0; //new frame starts new checksum
    _SHmessageChksumCalc = 0;

    // Zigbee Frame Delimiter
    _myZBframeRX.ZBfrmDelimiter = _ZBfrmBufferRX[ZB_FRM_OFFSET_DELMTR];  // ZB Frame Delimiter

    // Zigbee 16bit BE Frame Length
    ptrZBfrmLength[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENH];  //ZB frame length BE MSbyte
    ptrZBfrmLength[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_LENL];  //ZB frame length BE LSbyte

    // start calculating RX frame checksum for comparison
    // checksum is calculated on bytes BETWEEN (not including) the Zigbee frame length and checksum byte offsets
    
    // Zigbee Frame Type
    _myZBframeRX.ZBfrmType = _ZBfrmBufferRX[ZB_FRM_OFFSET_FTYPE];  //ZB frame Type
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_FTYPE];
 
    // Zigbee 64bit BE Source addr
    ptrZBsaddr64H[3] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B7];  // ZB frame BE 64bit DADDR Hword MSbyte 7
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B7];

    ptrZBsaddr64H[2] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B6];  // ZB frame BE 64bit DADDR Hword byte 6
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B6];

    ptrZBsaddr64H[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B5];  // ZB frame BE 64bit DADDR Hword byte 5
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B5];

    ptrZBsaddr64H[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B4];  // ZB frame BE 64bit DADDR Hword LSbyte 4
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B4];
    
    ptrZBsaddr64L[3] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B3];  // ZB frame BE 64bit DADDR Lword MSbyte 3
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B3];

    ptrZBsaddr64L[2] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B2];  // ZB frame BE 64bit DADDR Lword byte 2
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B2];

    ptrZBsaddr64L[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B1];  // ZB frame BE 64bit DADDR Lword byte 1
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B1];

    ptrZBsaddr64L[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B0];  // ZB frame BE 64bit DADDR Lword LSbyte 0
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_TX_DADDR64B0];
    
    // Zigbee 16bit BE Source addr
    ptrZBsaddr16[1] = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16H];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16H];

    ptrZBsaddr16[0] = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16L];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_SADDR16L];

    // Zigbee Options byte
    _myZBframeRX.ZBfrmOptions = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_OPTIONS];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_OPTIONS];
            
    //// BEGIN SmartHome message payload bytes
	
    //// ZB Payload - SmartHome 16bit BE Node Destination Address
    ptrZBfrmPldSHdestAddr16[1] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H)];

    ptrZBfrmPldSHdestAddr16[0] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L)];

    SHmsgRX.SHdestID = _myZBframeRX.ZBfrmPayload.SHdestID;

    //// ZB Payload - SmartHome 16bit BE Node Source Address
    ptrZBfrmPldSHsrcAddr16[1] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H)];

    ptrZBfrmPldSHsrcAddr16[0] = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L)];

    SHmsgRX.SHsrcID = _myZBframeRX.ZBfrmPayload.SHsrcID;

    //// ZB Payload - SmartHome Message Type (CMD_INIT, ACK_REQ, CONFIRM, COMPLETE)
    _myZBframeRX.ZBfrmPayload.SHmsgType = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE)];

    SHmsgRX.SHmsgType = _myZBframeRX.ZBfrmPayload.SHmsgType;

    //// ZB Payload - SmartHome Command
    _myZBframeRX.ZBfrmPayload.SHcommand = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CMD)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CMD)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CMD)];

    SHmsgRX.SHcommand = _myZBframeRX.ZBfrmPayload.SHcommand;

    //// ZB Payload - SmartHome Status 2=H / 16bit StatusID BE Hbyte/MSbyte
    _myZBframeRX.ZBfrmPayload.SHstatusH = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_H)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_H)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_H)];

    SHmsgRX.SHstatusH = _myZBframeRX.ZBfrmPayload.SHstatusH;

    //// ZB Payload - SmartHome Status 1=L / 16bit StatusID BE Lbyte/LSbyte
    _myZBframeRX.ZBfrmPayload.SHstatusL = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_L)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_L)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_L)];

    SHmsgRX.SHstatusL = _myZBframeRX.ZBfrmPayload.SHstatusL;

    //// ZB Payload - SmartHome Status 0=StatusVal 8bits
    _myZBframeRX.ZBfrmPayload.SHstatusVal = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL)];

    SHmsgRX.SHstatusVal = _myZBframeRX.ZBfrmPayload.SHstatusVal;

    //// ZB Payload - SmartHome Reserved Bytes (for Future Use)
    _myZBframeRX.ZBfrmPayload.SHreserved1 = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_1)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_1)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_1)];

    SHmsgRX.SHreserved1 = _myZBframeRX.ZBfrmPayload.SHreserved1;

    _myZBframeRX.ZBfrmPayload.SHreserved2 = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_2)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_2)];
    _SHmessageChksumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_2)];

    SHmsgRX.SHreserved2 = _myZBframeRX.ZBfrmPayload.SHreserved2;

    //// ZB Payload - SmartHome Message Checksum (provided by the sender)
    _myZBframeRX.ZBfrmPayload.SHpayldChksum = _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CHKSUM)];
    _ZBfrmRXchkSumCalc += _ZBfrmBufferRX[(ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CHKSUM)];
    _SHmessageChksumCalc = (uint8_t)0xff - _SHmessageChksumCalc;

    SHmsgRX.SHpayldChksum = _myZBframeRX.ZBfrmPayload.SHpayldChksum;

    //// END of the SmartHome Message / Zigbee Frame Payload
	
    // END of Zigbee Frame bytes which are included inteh ZB Frame Checksum calculation
	
    // Zigbee Frame checksum
    _myZBframeRX.ZBfrmChksum = _ZBfrmBufferRX[ZB_FRM_OFFSET_RX_CHKSUM];
    _ZBfrmRXchkSumCalc = (uint8_t)0xff - _ZBfrmRXchkSumCalc;

    // Debug output to serial monitor
    _debugPrintZBframeStructRX();
    _debugPrintSHmsgRX();
}


// Send the Zigbee frame receive buffer to Serial port monitor for developer/debugging purposes
void SHzigbee::_debugPrintZBframeBufRX(void)
{
#ifdef DEBUG_ZB_RECEIVE
    uint16_t i=0;

    Serial.print("RX ZF Frame = ");
    for(i=0; i<ZB_RX_FRM_BYTES; i++)
    {
        Serial.print( _ZBfrmBufferRX[i], HEX );
	Serial.print(" ");
    }
    Serial.println("  <ZBbufRX>");
#endif // DEBUG_ZB_RECEIVE
}


// Send the Zigbee frame receive struct to Serial port monitor for developer/debugging purposes
void SHzigbee::_debugPrintZBframeStructRX(void)
{
#ifdef DEBUG_ZB_RECEIVE

//    Serial.print("_myZBframeRX.ZBfrmDelimiter (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmDelimiter, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmLength (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmLength, HEX);
//    Serial.print(" = (decimal) ");
    Serial.print("=");
    Serial.print(_myZBframeRX.ZBfrmLength, DEC);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmType (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmType, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBsaddr64High (hex) = ");
    Serial.print(_myZBframeRX.ZBsaddr64High, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBsaddr64Low (hex) = ");
    Serial.print(_myZBframeRX.ZBsaddr64Low, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBsaddr16 (hex) = ");
    Serial.print(_myZBframeRX.ZBsaddr16, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmOptions (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmOptions, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHdestID (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHdestID, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHsrcID (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHsrcID, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHmsgType (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHmsgType, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHcommand (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHcommand, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHstatusH (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHstatusH, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHstatusL (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHstatusL, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHstatusVal (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHstatusVal, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHreserved1 (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHreserved1, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHreserved2 (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHreserved2, HEX);
    Serial.print(" ");

//    Serial.print("_myZBframeRX.ZBfrmPayload.SHpayldChksum (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmPayload.SHpayldChksum, HEX);
//    Serial.print(" ?=? _SHmessageChksumCalc =  ");
    Serial.print("?=?");
    Serial.print(_SHmessageChksumCalc, HEX);
    Serial.print(" ");
 
//    Serial.print("_myZBframeRX.ZBfrmChksum (hex) = ");
    Serial.print(_myZBframeRX.ZBfrmChksum, HEX);
//    Serial.print(" ?=? _ZBfrmRXchkSumCalc =  ");
    Serial.print("?=?");
    Serial.print(_ZBfrmRXchkSumCalc, HEX);
    Serial.println(" <ZBstructRX>");
#endif // DEBUG_ZB_RECEIVE
}


void SHzigbee::_debugPrintSHmsgRX(void)
{
#ifdef DEBUG_ZB_RECEIVE

//    Serial.print("SHmsgRX.SHdestID (hex) = ");
    Serial.print(SHmsgRX.SHdestID, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHsrcID (hex) = ");
    Serial.print(SHmsgRX.SHsrcID, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHmsgType (hex) = ");
    Serial.print(SHmsgRX.SHmsgType, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHcommand (hex) = ");
    Serial.print(SHmsgRX.SHcommand, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHstatusH (hex) = ");
    Serial.print(SHmsgRX.SHstatusH, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHstatusL (hex) = ");
    Serial.print(SHmsgRX.SHstatusL, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHstatusVal (hex) = ");
    Serial.print(SHmsgRX.SHstatusVal, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHreserved1 (hex) = ");
    Serial.print(SHmsgRX.SHreserved1, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHreserved2 (hex) = ");
    Serial.print(SHmsgRX.SHreserved2, HEX);
    Serial.print(" ");

//    Serial.print("SHmsgRX.SHpayldChksum (hex) = ");
    Serial.print(SHmsgRX.SHpayldChksum, HEX);
//    Serial.print(" ?=? _SHmessageChksumCalc =  ");
    Serial.print("?=?");
    Serial.print(_SHmessageChksumCalc, HEX);
    Serial.println(" <SHmsgRX>");
 
#endif // DEBUG_ZB_RECEIVE
}

