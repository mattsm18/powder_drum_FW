
/*
Title: SerialHandler.h
Author: Matthew Smith 22173112
Date: 6/05/26
Purpose:
- Handle Serial Comms based off pre-defined serial frame protocol
- State machine to handle

PARAMETER:     | SOF | VERSION | MSG_ID | DIR | LEN | PAYLOAD | XOR CRC |
SIZE (BYTES):  |  1  |    1    |   1    |  1  |  1  |  0-255  |    1    |

*/


#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <Arduino.h>

///////////////////////////////
//*** Constant Defintions ***//
///////////////////////////////

#define SOF_BYTE                    0xAA
#define SERIAL_PROTOCOL_VERSION     0x01
#define MAX_PAYLOAD_BYTES           255
#define HEADER_SIZE_BYTES           5
#define TIMEOUT_MS                  100 
#define MAX_RETRIES                 3
#define DIR_PC_TO_MCU               0x01
#define DIR_MCU_TO_PC               0x02

// Callback typedefs
typedef void    (*SetCallback)(uint8_t parameter_id, float value);
typedef float (*GetCallback)(uint8_t parameter_id);

// Union definition of float -> IEEE 754
union FloatBytes {
    float f;
    uint8_t b[4];
};

////////////////////////
//*** Enumerations ***//
////////////////////////

// MessageID Lookup table
enum MsgID : uint8_t {
    MSG_CMD_SET     = 0x01,
    MSG_CMD_GET     = 0x02,
    MSG_STATUS      = 0x10,
    MSG_ACK         = 0xEE,
    MSG_NACK        = 0xEF,
    MSG_HEARTBEAT   = 0xFF,
};

// NACK error codes
enum NackError : uint8_t {
    ERR_VERSION_MISMATCH = 0x01,
    ERR_BAD_CRC          = 0x02,
    ERR_UNKNOWN_MSG      = 0x03,
    ERR_BAD_LEN          = 0x04,
};

// Packet representation
struct Packet {
    uint8_t version;
    uint8_t msg_id;
    uint8_t dir;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD_BYTES];
    uint8_t crc;
};

// Serial State Machine Representation
enum SerialState {
    IDLE,
    READ_HEADER,
    READ_PAYLOAD,
    VALIDATE,
    DISPATCH
};

////////////////////////////
//*** Class Definition ***//
////////////////////////////

class SerialHandler {
public:
    // Call on setup
    void begin(uint32_t baud_rate);

    // Call every loop
    void update();

    // State of Health Updates
    void sendParameter(uint8_t parameter_id, float value);
    void sendStatus(uint8_t status_code);
    void sendHeartbeat();

    // Callbacks
    void onSet(SetCallback cb) { _setCb = cb; }
    void onGet(GetCallback cb) { _getCb = cb; }

private:
    // State machine
    SerialState _state;

    // Callbacks
    SetCallback _setCb = nullptr;
    GetCallback _getCb = nullptr;
    
    // State handler implementations
    void _handleIdle();
    void _handleReadHeader();
    void _handleReadPayload();
    void _handleValidate();
    void _handleDispatch();
    
    // RX (Incoming data)
    Packet _rxPacket;
    uint8_t _rxBuffer[HEADER_SIZE_BYTES + MAX_PAYLOAD_BYTES + 1];
    uint8_t _bytesRead;

    // TX (Outgoing data)
    void _sendPacket(uint8_t msg_id, uint8_t dir, const uint8_t* payload, uint8_t len);
    void _sendACK(uint8_t ack_msg_id);
    void _sendNACK(uint8_t nack_msg_id, NackError error);

    void _onCmdSet(const Packet& pkt);
    void _onCmdGet(const Packet& pkt);

    // CRC
    uint8_t _computeCRC(const uint8_t* data, uint8_t len);

    // Timeout
    uint32_t _lastByteTime;

};


#endif