#ifndef SH_ZIGBEE_H
#define SH_ZIGBEE_H

// cross-platform consistent integer types
#include "stdint.h"

#include "SmartHome_Zigbee_defs.h"

#include "shLinuxSerialPort.h"


class SHzigbee
{
    public:
	    volatile uint8_t    ZBinFrameRX;        // is a recognizable Zigbee frame coming in or not yet
	    volatile uint8_t    ZBnewFrameRXed;     // Has a new Zigbee frame been received/completed
        ZBframeRX     myZBframeRX;                //A Zigbee RX RVCD type API frame struct instance to work with
        ZBframeTX     myZBframeTX;                //A Zigbee TX REQ type API frame struct instance to work with
    	volatile uint8_t    newSHmsgRX;         // Has a new SmartHome message been received/completed
        volatile SHpayload  SHmsgRX;            // SmartHome message received
        volatile SHpayload  SHmsgTX;            // SmartHome message to send

//        const char* SH_SERVER_SERIAL_PORT_NAME = "/dev/ttyUSB0";
//        shSerialPort  shSerialPortZigbee = shSerialPort(SH_SERVER_SERIAL_PORT_NAME);
        shSerialPort  shSerialPortZigbee; // = shSerialPort();

//    	SHzigbee(const char* portName); //constructor
    	SHzigbee(void); //constructor

    	~SHzigbee(void); //destructor

//        bool start(unsigned int baud_rate);
        bool start(const char* portName, unsigned int baud_rate);

	    // Transmit a Zigbee TX REQ type API frame, previously prepared with SmartHome payload content
        uint8_t zbXmitAPIframe(void);

        // Prepare the TX frame message payload to be sent
        void prepareTXmsg( uint16_t prepSHdestID,     // Dest ID
                           uint16_t prepSHsrcID,      // Source ID
                           uint8_t  prepSHmsgType,    // Msg Type
                           uint8_t  prepSHcommand,    // CMD
                           uint8_t  prepSHstatusH,    // Status/StatusID High byte
                           uint8_t  prepSHstatusL,    // Status/StatusID Low byte
                           uint8_t  prepSHstatusVal   // Status value (8bit)
                         );

	// read back the current value of this TX message type (of SmartHome Message types)
        uint8_t getMsgTypeTX(void);

	// Receive (attempt to) a Zigbee RX RCVD type API frame, previously prepared with SmartHome payload content
	uint8_t zbRcvAPIframe(void);

    private:
	// Prefix ALL private items with "_" (underscore) as coding style to help indicate public vs private

        uint8_t _ZBfrmBufferTX[ZB_TX_FRM_BYTES];   // byte array buffer to dump into Serial.write()

        uint8_t  _ZBfrmBufferRX[ZB_RX_FRM_BYTES];  // byte array buffer to dump into FROM Serial.read()
        uint16_t _ZBoffsetRXbuff;                  // offset index into the _ZBfrmBufferRX Zigbee Frame buffer
	    uint8_t  _ZBfrmRXchkSumCalc;               // locally calculated checksum of the ZB API frame
        uint8_t  _SHmessageChksumCalc;             // locally calculated checksum of the SmartHome Message payload
//        uint8_t  _ZBinFrameRX;

//        const char* _SH_SERVER_SERIAL_PORT_NAME = "/dev/ttyUSB0";
//        //const wxString _SH_SERVER_SERIAL_PORT_NAME = "/dev/ttyUSB0";
//        const int   _SH_SERVER_SERIAL_BAUD_RATE = 9600;
//        const int   _SH_SERVER_SERIAL_CHAR_SIZE =  8;

//        shSerialPort  _shSerialPortZigbee(_SH_SERVER_SERIAL_PORT_NAME);
//        shSerialPort  _shSerialPortZigbee( (const char*)"/dev/ttyUSB0" );
//        shSerialPort  _shSerialPortZigbee = shSerialPort(_SH_SERVER_SERIAL_PORT_NAME);
//        shSerialPort  _shSerialPortZigbee;

	    void     _initXmitAPIframe(void);
        uint8_t  _calcChkSum8(uint8_t ui8);
        uint8_t  _calcChkSum16(uint16_t ui16);
        uint8_t  _calcChkSum32(uint32_t ui32);
	    void     _parseZBbufferRX(void);
	    void     _debugPrintZBframeBufRX(void);
	    void     _debugPrintZBframeStructRX(void);
        void     _debugPrintSHmsgRX(void);
        void     _debugPrintZBframeBufTX(void);

}; // end class SHzigbee

#endif // SH_ZIGBEE_H
