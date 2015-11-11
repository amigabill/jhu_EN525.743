// Include Arduino standard libraries
#include "Arduino.h"

#include "../ByteSwap/ByteSwap.h"

#include "SmartHome_Zigbee.h"


// constructor
SHzigbee::SHzigbee()
{
	initXmitAPIframe();
        //initSHmsgRX();
	
        _ZBoffsetRXbuff = ZB_FRM_OFFSET_DELMTR; //ZB_START_DELIMITER; //Delimiter byte is offset 0 into RX buffer	

	ZBnewFrameRXed = NO;
	newSHmsgRX = NO;
	ZBinFrameRX = NO;
}



// Initialize the TX API frame with values we will use as standard
void SHzigbee::initXmitAPIframe(void)
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
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmType);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_FID]          = _myZBframeTX.ZBfrmID; //(uint8_t)0x01;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmID);

    // 64bit Zigbee addr
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B7] = ptrZBdaddr64H[3]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B6] = ptrZBdaddr64H[2]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B5] = ptrZBdaddr64H[1]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B4] = ptrZBdaddr64H[0]; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += calcChkSum32(_myZBframeTX.ZBdaddr64High);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B3] = ptrZBdaddr64L[3]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B2] = ptrZBdaddr64L[2]; //(uint8_t)0x00;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B1] = ptrZBdaddr64L[1]; //(uint8_t)0xff;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR64B0] = ptrZBdaddr64L[0]; //(uint8_t)0xff;
    _myZBframeTX.ZBfrmChksum += calcChkSum32(_myZBframeTX.ZBdaddr64Low);

    // 16bit Zigbee addr
    uint8_t *ptrZBdaddr16 = (uint8_t *)&(_myZBframeTX.ZBdaddr16);
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR16H] = ptrZBdaddr16[1]; //(uint8_t)0xff;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_DADDR16L] = ptrZBdaddr16[0]; //(uint8_t)0xfe;
    _myZBframeTX.ZBfrmChksum += calcChkSum16(_myZBframeTX.ZBdaddr16);

    // Zigbee Radius and Options bytes
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_BRADIUS] = _myZBframeTX.ZBfrmRadius; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmRadius);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_OPTIONS] = _myZBframeTX.ZBfrmOptions; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmOptions);


    // SmartHome message payload bytes
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H]  = ptrZBfrmPldSHdestAddr16[1]; //(uint8_t)0xab;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L]  = ptrZBfrmPldSHdestAddr16[0]; //(uint8_t)0xcd;
    _myZBframeTX.ZBfrmChksum += calcChkSum16(_myZBframeTX.ZBfrmPayload.SHdestID);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H]   = ptrZBfrmPldSHsrcAddr16[1]; //(uint8_t)0xf0;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L]   = ptrZBfrmPldSHsrcAddr16[0]; //(uint8_t)0x0d;
    _myZBframeTX.ZBfrmChksum += calcChkSum16(_myZBframeTX.ZBfrmPayload.SHsrcID);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE]   = _myZBframeTX.ZBfrmPayload.SHmsgType; //(uint8_t)0x01;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHmsgType);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_CMD]        = _myZBframeTX.ZBfrmPayload.SHcommand; //(uint8_t)0x01;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHcommand);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_H]   = _myZBframeTX.ZBfrmPayload.SHstatusH; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusH);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_L]   = _myZBframeTX.ZBfrmPayload.SHstatusL; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusL);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL] = _myZBframeTX.ZBfrmPayload.SHstatusVal; //(uint8_t)0x00;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusVal);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_RESVD_1]    = _myZBframeTX.ZBfrmPayload.SHreserved1; //(uint8_t)0x54;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved1);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_RESVD_2]    = _myZBframeTX.ZBfrmPayload.SHreserved2; //(uint8_t)0x58;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved2);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_CHKSUM]     = _myZBframeTX.ZBfrmPayload.SHpayldChksum; //(uint8_t)0xdc;
    _myZBframeTX.ZBfrmChksum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHpayldChksum);


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


uint8_t SHzigbee::calcChkSum8(uint8_t ui8)
{
//    Serial.print(ui8, HEX);
//    Serial.print(" ");
    return(ui8);
}

uint8_t SHzigbee::calcChkSum16(uint16_t ui16)
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

uint8_t SHzigbee::calcChkSum32(uint32_t ui32)
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
    tmpChkSum += calcChkSum16(_myZBframeTX.ZBfrmPayload.SHdestID);
    
    _myZBframeTX.ZBfrmPayload.SHsrcID     = prepSHsrcID; //BYTESWAP16(prepSHsrcID);
    tmpChkSum += calcChkSum16(_myZBframeTX.ZBfrmPayload.SHsrcID);

    _myZBframeTX.ZBfrmPayload.SHmsgType   = prepSHmsgType;
    tmpChkSum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHmsgType);

    _myZBframeTX.ZBfrmPayload.SHcommand   = prepSHcommand;
    tmpChkSum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHcommand);

    _myZBframeTX.ZBfrmPayload.SHstatusH   = prepSHstatusH;
    tmpChkSum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusH);

    _myZBframeTX.ZBfrmPayload.SHstatusL   = prepSHstatusL;
    tmpChkSum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusL);

    _myZBframeTX.ZBfrmPayload.SHstatusVal = prepSHstatusVal;
    tmpChkSum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHstatusVal);

    _myZBframeTX.ZBfrmPayload.SHreserved1 = (uint8_t)'T';
    tmpChkSum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved1);

    _myZBframeTX.ZBfrmPayload.SHreserved2 = (uint8_t)'X';
    tmpChkSum += calcChkSum8(_myZBframeTX.ZBfrmPayload.SHreserved2);

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
uint8_t SHzigbee::zbRcvAPIframe(void)
{
    // use some pointer to unsigned int 8 (byte) to access parts of larger datatypes a byte at a time during Zigbee frame receive
    uint8_t *ptrZBfrmLength  = (uint8_t *)&(_myZBframeRX.ZBfrmLength);
    uint8_t *ptrZBsaddr64H   = (uint8_t *)&(_myZBframeRX.ZBsaddr64High);
    uint8_t *ptrZBsaddr64L   = (uint8_t *)&(_myZBframeRX.ZBsaddr64Low);
    uint8_t *ptrZBsaddr16    = (uint8_t *)&(_myZBframeRX.ZBsaddr16);
    uint8_t *ptrZBfrmPldSHdestAddr16  = (uint8_t *)&(_myZBframeRX.ZBfrmPayload.SHdestID);
    uint8_t *ptrZBfrmPldSHsrcAddr16   = (uint8_t *)&(_myZBframeRX.ZBfrmPayload.SHsrcID);
    uint8_t *ptrSHdestAddr16          = (uint8_t *)&(SHmsgRX.SHdestID); // Pointer into SHmsgRX received SmartHome message struct
    uint8_t *ptrSHsrcAddr16           = (uint8_t *)&(SHmsgRX.SHsrcID);  // Pointer into SHmsgRX received SmartHome message struct

    uint8_t ZB_frm_byte = 0; // byte received in uart RX by Serial.read
//    uint8_t ZBchksumFromSender = 0; // checksum sent to us for comparison


    while (Serial.available() > 0)
    {

        // Read a byte from uart RX buffer
        ZB_frm_byte = Serial.read();
	    // store whatever current byte into RX buffer and increment offset index for next byte
            _ZBfrmBufferRX[_ZBoffsetRXbuff] = ZB_frm_byte; 


	Serial.print(ZB_frm_byte, HEX);
	Serial.print(" ");

        if ((ZBinFrameRX == NO) && (ZB_frm_byte == ZB_START_DELIMITER))
        {
            // beginning a new frame
            ZBinFrameRX = YES;
            _ZBoffsetRXbuff = 0; //ZB_START_DELIMITER; //Delimiter byte is offset 0 into RX buffer
            _myZBframeRX.ZBfrmChksum = 0; //new frame starts new checksum
            _ZBfrmBufferRX[_ZBoffsetRXbuff] = ZB_frm_byte; //(uint8_t)0x7e;  // ZB Frame Delimiter
            _ZBfrmRXchkSumCalc = 0;

	    Serial.println("Got new RX frame Delimiter");
        }
        else if (ZBinFrameRX == NO) // and new byte, which is NOT ZB_START_DELIMITER
        {
            // NOT already in a frame, and NOT starting a new frame here, ignore unknown bytes in RX
            _ZBoffsetRXbuff = 0;
            return (0);
        }
        else  // ZBinFrameRX == YES so are in a frame
        {
            if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_LENH)  //ZB frame BE lenH
            {
	        ptrZBfrmLength[1] = ZB_frm_byte; //lenH
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_LENL)  //ZB frame BE lenL
            {
	        ptrZBfrmLength[0] = ZB_frm_byte; //lenL
            }	    

	    // checksum is calculated on bytes BETWEEN (not including) the Zigbee frame length and checksum byte offsets

            // 64bit Zigbee addr
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_FTYPE)  //ZB frame Type
            {
                _ZBfrmBufferRX[_ZBoffsetRXbuff] = ZB_frm_byte; 
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B7)  //ZB frame 64bit DADDR BE Hword MSbyte 7
            {
	        ptrZBsaddr64H[3] = ZB_frm_byte; // BE MSbyte of the High 32bit portion
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B6)  //ZB frame 64bit DADDR Hword byte 6
            {
	        ptrZBsaddr64H[2] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B5)  //ZB frame 64bit DADDR Hword byte 5
            {
	        ptrZBsaddr64H[1] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B4)  //ZB frame 64bit DADDR Hword LSbyte 4
            {
	        ptrZBsaddr64H[0] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B3)  //ZB frame 64bit DADDR Lword MSbyte 3
            {
    	        ptrZBsaddr64L[3] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B2)  //ZB frame 64bit DADDR Lword byte 2
            {
	        ptrZBsaddr64L[2] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B1)  //ZB frame 64bit DADDR Lword byte 1
            {
	        ptrZBsaddr64L[1] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR64B0)  //ZB frame 64bit DADDR Lword LSbyte 0
            {
	        ptrZBsaddr64L[0] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    

            // 16bit Zigbee addr
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR16H)  //ZB frame 16bit DADDR MSbyte 1
            {
	        ptrZBsaddr16[1] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_SADDR16L)  //ZB frame 16bit DADDR LSbyte 0
            {
	        ptrZBsaddr16[0] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
	
            // Zigbee Options byte
            else if(_ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_OPTIONS)  //ZB frame Options
            {
	        _myZBframeRX.ZBfrmOptions = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
	
            // SmartHome message payload bytes
	
	    // ZB Payload - 16bit SmartHome Node Destination Address BE
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H) )  //SH Msg Dest addr BE MSbyte 1
            {
	        ptrZBfrmPldSHdestAddr16[1] = ZB_frm_byte;
		ptrSHdestAddr16[1] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
	        _SHmessageChksumCalc = 0; //init for this Zigbee frame and the SmartHome message contained within
	        _SHmessageChksumCalc += ZB_frm_byte;
            }	    
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L) )  //SH Msg Dest addr BE LSbyte 0
            {
	        ptrZBfrmPldSHdestAddr16[0] = ZB_frm_byte;
		ptrSHdestAddr16[0] = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
            }	    

	    // ZB Payload - 16bit SmartHome Node Source Address BE
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H) )  //SH Msg Src addr MSbyte 1
            {
		ptrZBfrmPldSHsrcAddr16[1] = ZB_frm_byte;
	        ptrSHsrcAddr16[1] = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L) )  //SH Msg Src addr LSbyte 0
            {
		ptrZBfrmPldSHsrcAddr16[0] = ZB_frm_byte;
	        ptrSHsrcAddr16[0] = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
	
	    // ZB Payload - SmartHome Message Type (CMD_INIT, ACK_REQ, CONFIRM, COMPLETE)
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_MSG_TYPE) )  //SH Msg Type
            {
	        _myZBframeRX.ZBfrmPayload.SHmsgType = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
		SHmsgRX.SHmsgType = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
            }	    
	
	    // ZB Payload - SmartHome Command
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CMD) )  //SH Msg Command
            {
	        _myZBframeRX.ZBfrmPayload.SHcommand = ZB_frm_byte;
		SHmsgRX.SHcommand = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
	
    	    // ZB Payload - SmartHome Status 2=H / 16bit StatusID BE Hbyte/MSbyte
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_H) )  //SH Msg StatusH
            {
	        _myZBframeRX.ZBfrmPayload.SHstatusH = ZB_frm_byte;
		SHmsgRX.SHstatusH = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    

	    // ZB Payload - SmartHome Status 1=L / 16bit StatusID BE Lbyte/LSbyte
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_L) )  //SH Msg StatusL
            {
	        _myZBframeRX.ZBfrmPayload.SHstatusL = ZB_frm_byte;
		SHmsgRX.SHstatusL = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
	
	    // ZB Payload - SmartHome Status 0=StatusVal 8bits
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_STATUS_VAL) )  //SH Msg StatusVal
            {
	        _myZBframeRX.ZBfrmPayload.SHstatusVal = ZB_frm_byte;
		SHmsgRX.SHstatusVal = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
            }	    
	
	    // ZB Payload - SmartHome Reserved Bytes (for Future Use)
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_1) )  //SH Msg Reserved1
            {
	        _myZBframeRX.ZBfrmPayload.SHreserved1 = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
		SHmsgRX.SHreserved1 = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
            }	    
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_RESVD_2) )  //SH Msg Reserved1
            {
	        _myZBframeRX.ZBfrmPayload.SHreserved2 = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;
		SHmsgRX.SHreserved2 = ZB_frm_byte;
	        _SHmessageChksumCalc += ZB_frm_byte;
            }	    

	    // ZB Payload - SmartHome Message Checksum (provided by the sender)
            else if( _ZBoffsetRXbuff == (ZB_FRM_OFFSET_RX_PAYLOAD + SH_MSG_OFFSET_CHKSUM) )  //SH Msg Checksum
            {
	        _myZBframeRX.ZBfrmPayload.SHpayldChksum = ZB_frm_byte;
		SHmsgRX.SHpayldChksum = ZB_frm_byte;
                _ZBfrmRXchkSumCalc += ZB_frm_byte;

	        _SHmessageChksumCalc = 0xff - _SHmessageChksumCalc;
	        // does the SmartHome Message Checksum match?
		
	Serial.print("SHchecksum  _ZBfrmBufferRX[");
	Serial.print(_ZBoffsetRXbuff, HEX);
	Serial.print("]=");
	Serial.print(_ZBfrmBufferRX[_ZBoffsetRXbuff], HEX);
	Serial.println();

            }	    
	
	    // END of the SmartHome Message / Zigbee Frame Payload
	
	    // END of Zigbee Frame bytes which are included inteh ZB Frame Checksum calculation
	
	    // ZB Frame checksum
            else if( _ZBoffsetRXbuff == ZB_FRM_OFFSET_RX_CHKSUM )  //Zigbee Frame Checksum
            {
	        _myZBframeRX.ZBfrmChksum = ZB_frm_byte;

		// finalize the Zigbee frame checksum value
                _ZBfrmRXchkSumCalc = 0xff - _ZBfrmRXchkSumCalc;

	        // does the ZB frame checksum match?
		
		
		// indicate no longer receiving bytes for a Zigbee frame
                ZBinFrameRX = NO;

		// and that a received SmartHome Message is ready to process
                newSHmsgRX = YES;

		Serial.print("ZBchecksum=");
                Serial.print(_myZBframeRX.ZBfrmChksum, HEX);
		Serial.print(" ?=? _ZBfrmRXchkSumCalc=");
		Serial.print(_ZBfrmRXchkSumCalc, HEX);
		Serial.print(" ; ZBinFrameRX=");
		Serial.print(ZBinFrameRX, HEX);
		Serial.println();

		//debugPrintZBframeRX();
#if 1
                Serial.println("");
                uint16_t i=0;
                for(i-0; i<ZB_RX_FRM_BYTES; i++)
                {
                    Serial.print( _ZBfrmBufferRX[i], HEX );
                    Serial.print(" ");
                }
                Serial.println();
#endif    
                Serial.print("newSHmsgRX=");
		Serial.print(newSHmsgRX, HEX);
                Serial.println();
                Serial.println("Finished receiving RXframe");
            }

        }	    
#if 0
	Serial.print("_ZBfrmBufferRX[");
	Serial.print(_ZBoffsetRXbuff, HEX);
	Serial.print("]=");
	Serial.print(_ZBfrmBufferRX[_ZBoffsetRXbuff], HEX);
	Serial.println();
#endif

	// increment offset into the RX buffer of bytes
        _ZBoffsetRXbuff += 1;
    }
}
    
#if 0
void debugPrintZBframeRX(void)
{
    uint16_t i=0;

    Serial.print("RX ZF Frame = ");
    for(i=0; i<ZB_RX_FRM_BYTES; i++)
    {
        Serial.print( _ZBfrmBufferRX[i], HEX );
	Serial.print(" ");
    }
    Serial.println();
}
#endif


