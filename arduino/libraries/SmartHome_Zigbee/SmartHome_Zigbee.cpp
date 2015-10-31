// Include Arduino standard libraries
#include "Arduino.h"

#include "../ByteSwap/ByteSwap.h"

#include "SmartHome_Zigbee.h"


SHzigbee::SHzigbee()
{
	initXmitAPIframe();
}


// Initialize the TX API frame with values we will use as staandard
void SHzigbee::initXmitAPIframe(void)
{
    _myZBframeTX.ZBfrmDelimiter = ZB_START_DELIMITER;
    _myZBframeTX.ZBfrmLength    = ZB_TX_FRM_LEN; 
    _myZBframeTX.ZBfrmType      = ZB_FRAME_TYPE_TX_REQ;
    _myZBframeTX.ZBfrmID        = (uint8_t)1; // always use ID of 1
    _myZBframeTX.ZBdaddr64High  = ZB_64ADDR_BCAST_HIGH;
    _myZBframeTX.ZBdaddr64Low   = ZB_64ADDR_BCAST_LOW;
    _myZBframeTX.ZBdaddr16      = ZB_16ADDR_BCAST;
    _myZBframeTX.ZBfrmRadius    = ZB_BCAST_RADIUS;
    _myZBframeTX.ZBfrmOptions   = ZB_OPTIONS;
}


// Transmit a Zigbee API TX Request Frame
// Each Zigbee message frame has a 12byte payload, the SmartHome Message,
// so length=23=0x17 bytes, total frame=27bytes
uint8_t SHzigbee::zbXmitAPIframe(void)
{
    uint8_t *ptrZBfrmLength = (uint8_t *)&(_myZBframeTX.ZBfrmLength);
    uint8_t *ptrZBdaddr64H = (uint8_t *)&(_myZBframeTX.ZBdaddr64High);
    uint8_t *ptrZBdaddr64L = (uint8_t *)&(_myZBframeTX.ZBdaddr64Low);
    uint8_t *ptrSHdestAddr16 = (uint8_t *)&(_myZBframeTX.ZBfrmPayload.SHdestID);
    uint8_t *ptrSHsrcAddr16 = (uint8_t *)&(_myZBframeTX.ZBfrmPayload.SHsrcID);


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
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_H]  = ptrSHdestAddr16[1]; //(uint8_t)0xab;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_DEST_L]  = ptrSHdestAddr16[0]; //(uint8_t)0xcd;
    _myZBframeTX.ZBfrmChksum += calcChkSum16(_myZBframeTX.ZBfrmPayload.SHdestID);

    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_H]   = ptrSHsrcAddr16[1]; //(uint8_t)0xf0;
    _ZBfrmBufferTX[ZB_FRM_OFFSET_TX_PAYLOAD + SH_MSG_OFFSET_ID_SRC_L]   = ptrSHsrcAddr16[0]; //(uint8_t)0x0d;
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

    Serial.write(_ZBfrmBufferTX, ZB_TX_FRM_BYTES);

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

    Serial.println("");
    Serial.print("Finished sending TXframe");
}


uint8_t SHzigbee::calcChkSum8(uint8_t ui8)
{
    Serial.print(ui8, HEX);
    Serial.print(" ");
    return(ui8);
}

uint8_t SHzigbee::calcChkSum16(uint16_t ui16)
{
    uint8_t *ptrUI8AsUi16 = (uint8_t *)&ui16;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi16[1];
    Serial.print(ptrUI8AsUi16[1], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi16[0];
    Serial.print(ptrUI8AsUi16[0], HEX);
    Serial.print(" ");

    return(tmpChkSum);
}

uint8_t SHzigbee::calcChkSum32(uint32_t ui32)
{
    uint8_t *ptrUI8AsUi32 = (uint8_t *)&ui32;
    uint8_t tmpChkSum = 0;

    tmpChkSum += ptrUI8AsUi32[3];
    Serial.print(ptrUI8AsUi32[3], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[2];
    Serial.print(ptrUI8AsUi32[2], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[1];
    Serial.print(ptrUI8AsUi32[1], HEX);
    Serial.print(" ");
    tmpChkSum += ptrUI8AsUi32[0];
    Serial.print(ptrUI8AsUi32[0], HEX);
    Serial.print(" ");

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

    #if 1
    Serial.print("<mcs=");
    Serial.print(_myZBframeTX.ZBfrmPayload.SHpayldChksum, HEX);
    Serial.print("> ");
    #endif
}


