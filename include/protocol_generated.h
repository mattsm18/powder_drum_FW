// ============================================================
// AUTO-GENERATED FILE — DO NOT EDIT BY HAND
// Source:    config\pd_comms_protocol_v1.1.json (protocol v1.1)
// Generator: scripts/generate_protocol.py
// Regenerate with: python scripts/generate_protocol.py
// ============================================================

#ifndef PROTOCOL_GENERATED_H
#define PROTOCOL_GENERATED_H

#include <stdint.h>
#include <avr/pgmspace.h>

// ---- Frame constants ----
#define SOF_BYTE                    0xAA
#define SERIAL_PROTOCOL_VERSION     0x02
#define HEADER_SIZE_BYTES           5
#define MAX_PAYLOAD_BYTES           255
#define TIMEOUT_MS                  100
#define MAX_RETRIES                 3

// ---- Direction ----
#define DIR_PC_TO_MCU                0x01
#define DIR_MCU_TO_PC                0x02

// ---- Message IDs ----
enum MsgID : uint8_t {
    MSG_CMD_SET      = 0x01,
    MSG_CMD_GET      = 0x02,
    MSG_STATUS       = 0x10,
    MSG_ACK          = 0xEE,
    MSG_NACK         = 0xEF,
    MSG_HEARTBEAT    = 0xFF,
};

// ---- NACK error codes ----
enum NackError : uint8_t {
    ERR_VERSION_MISMATCH   = 0x01,
    ERR_BAD_CRC            = 0x02,
    ERR_UNKNOWN_MSG        = 0x03,
    ERR_BAD_LEN            = 0x04,
    ERR_BAD_DIRECTION      = 0x05,
    ERR_UNKNOWN_PARAM      = 0x06,
    ERR_READ_ONLY          = 0x07,
    ERR_WRITE_ONLY         = 0x08,
};

// ---- Parameter access flags (bitmask) ----
#define PARAM_ACCESS_R 0x01
#define PARAM_ACCESS_W 0x02

// ---- Parameter logical types ----
// NB: every parameter is still carried on the wire as a 4-byte IEEE-754
// float (see SerialHandler::sendParameter / MSG_CMD_SET payload layout).
// `type` here is metadata for validation/UI only, not the wire encoding.
enum ParamType : uint8_t {
    PARAM_TYPE_FLOAT = 0,
    PARAM_TYPE_INT   = 1,
    PARAM_TYPE_BOOL  = 2,
};

// ---- Parameter IDs ----
// Use these instead of magic hex when binding parameters in main.cpp.
namespace ParamID {
    constexpr uint8_t PROTOCOLVERSION            = 0x00; // Protocol Version [system]  (read-only)
    constexpr uint8_t SETPOINT                   = 0x01; // Setpoint rad/s [motor]
    constexpr uint8_t ACCELRATE                  = 0x02; // Accel Rate rad/s^2 [motor]
    constexpr uint8_t RAMPEDSETPOINT             = 0x03; // Ramped Setpoint rad/s [motor]  (read-only)
    constexpr uint8_t ENCODERANGULARVELOCITY     = 0x10; // Angular Velocity rad/s [motor]  (read-only)
    constexpr uint8_t KP                         = 0x20; // Kp [motor]
    constexpr uint8_t KI                         = 0x21; // Ki [motor]
    constexpr uint8_t KD                         = 0x22; // Kd [motor]
    constexpr uint8_t LIGHTS                     = 0x30; // Lights [io]
}

// ---- Parameter metadata table ----
// SerialHandler uses this to reject SET on read-only params and GET/SET on
// unknown ids generically, without per-parameter code. Lives in flash (PROGMEM)
// since the AVR target's SRAM is scarce.
struct ParamMeta {
    uint8_t id;
    uint8_t type;
    uint8_t access;
};

const ParamMeta PARAM_TABLE[] PROGMEM = {
    { ParamID::PROTOCOLVERSION, PARAM_TYPE_FLOAT, PARAM_ACCESS_R },
    { ParamID::SETPOINT, PARAM_TYPE_FLOAT, PARAM_ACCESS_R | PARAM_ACCESS_W },
    { ParamID::ACCELRATE, PARAM_TYPE_FLOAT, PARAM_ACCESS_R | PARAM_ACCESS_W },
    { ParamID::RAMPEDSETPOINT, PARAM_TYPE_FLOAT, PARAM_ACCESS_R },
    { ParamID::ENCODERANGULARVELOCITY, PARAM_TYPE_FLOAT, PARAM_ACCESS_R },
    { ParamID::KP, PARAM_TYPE_FLOAT, PARAM_ACCESS_R | PARAM_ACCESS_W },
    { ParamID::KI, PARAM_TYPE_FLOAT, PARAM_ACCESS_R | PARAM_ACCESS_W },
    { ParamID::KD, PARAM_TYPE_FLOAT, PARAM_ACCESS_R | PARAM_ACCESS_W },
    { ParamID::LIGHTS, PARAM_TYPE_BOOL, PARAM_ACCESS_R | PARAM_ACCESS_W },
};

#define PARAM_TABLE_LEN (sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]))

#endif // PROTOCOL_GENERATED_H
