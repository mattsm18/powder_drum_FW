#include "SerialHandler.h"

///////////////////////////////////////////////////////////////
// CORE FUNCTIONALITY & STATE MACHINE
///////////////////////////////////////////////////////////////

void SerialHandler::begin(uint32_t baud_rate) {
    Serial.begin(baud_rate);
    _state        = IDLE;
    _bytesRead    = 0;
    _lastByteTime = 0;
    memset(&_rxPacket, 0, sizeof(_rxPacket));
}

void SerialHandler::update() {
    // Mid-packet timeout guard — reset if we stall between bytes
    if (_state != IDLE && (millis() - _lastByteTime > TIMEOUT_MS)) {
        _state     = IDLE;
        _bytesRead = 0;
        memset(_rxBuffer, 0, sizeof(_rxBuffer)); // clear rx buffer on timeout
        return;
    }

    switch (_state) {
        case IDLE:         _handleIdle();        break;
        case READ_HEADER:  _handleReadHeader();  break;
        case READ_PAYLOAD: _handleReadPayload(); break;
        case VALIDATE:     _handleValidate();    break;
        case DISPATCH:     _handleDispatch();    break;
    }
}

///////////////////////////////////////////////////////////////
// STATE HANDLING FUNCTIONS
///////////////////////////////////////////////////////////////

void SerialHandler::_handleIdle() {
    // Hunt for SOF byte — discard everything else
    if (Serial.available() < 1) return;

    uint8_t byte = Serial.read();
    if (byte == SOF_BYTE) {
        _rxBuffer[0]  = byte;
        _bytesRead    = 1;
        _lastByteTime = millis();
        _state        = READ_HEADER;
    }
}

void SerialHandler::_handleReadHeader() {
    while (Serial.available() && _bytesRead < HEADER_SIZE_BYTES) {
        _rxBuffer[_bytesRead++] = Serial.read();
        _lastByteTime = millis();
    }

    if (_bytesRead < HEADER_SIZE_BYTES) return;

    _rxPacket.version = _rxBuffer[1];
    _rxPacket.msg_id  = _rxBuffer[2];
    _rxPacket.dir     = _rxBuffer[3];
    _rxPacket.len     = _rxBuffer[4];

    // Reject mismatched version immediately
    if (_rxPacket.version != SERIAL_PROTOCOL_VERSION) {
        _sendNACK(_rxPacket.msg_id, ERR_VERSION_MISMATCH);
        _state     = IDLE;
        _bytesRead = 0;
        return;
    }

    // Reject mismatched length immediately
    if (_rxPacket.len > MAX_PAYLOAD_BYTES) {
        _sendNACK(_rxPacket.msg_id, ERR_BAD_LEN);
        _state     = IDLE;
        _bytesRead = 0;
        return;
    }

    _state = (_rxPacket.len > 0) ? READ_PAYLOAD : VALIDATE;
}

void SerialHandler::_handleReadPayload() {
    uint8_t payloadStart = HEADER_SIZE_BYTES;

    while (Serial.available() && _bytesRead < HEADER_SIZE_BYTES + _rxPacket.len) {
        _rxBuffer[_bytesRead++] = Serial.read();
        _lastByteTime = millis();
    }

    if (_bytesRead < HEADER_SIZE_BYTES + _rxPacket.len) return; // wait for more

    // Copy payload out of raw buffer
    memcpy(_rxPacket.payload, &_rxBuffer[payloadStart], _rxPacket.len);
    _state = VALIDATE;
}

void SerialHandler::_handleValidate() {
    // Wait for the CRC byte
    if (Serial.available() < 1) return;

    uint8_t received_crc = Serial.read();

    // CRC covers everything before the CRC byte
    uint8_t computed_crc = _computeCRC(_rxBuffer, HEADER_SIZE_BYTES + _rxPacket.len);

    if (received_crc != computed_crc) {
        _sendNACK(_rxPacket.msg_id, ERR_BAD_CRC);
        _state     = IDLE;
        _bytesRead = 0;
        return;
    }

    _state = DISPATCH;
}

void SerialHandler::_handleDispatch() {

    switch (_rxPacket.msg_id) {
        case MSG_CMD_SET:   _onCmdSet(_rxPacket);   break;
        case MSG_CMD_GET:   _onCmdGet(_rxPacket);   break;
        case MSG_HEARTBEAT: sendHeartbeat();        break;
        case MSG_ACK:       /* handle if needed */  break;
        default:
            _sendNACK(_rxPacket.msg_id, ERR_UNKNOWN_MSG);
            break;
    }

    // Always reset for next packet
    _state     = IDLE;
    _bytesRead = 0;
    memset(&_rxPacket, 0, sizeof(_rxPacket));
    memset(_rxBuffer,  0, sizeof(_rxBuffer)); 
}

///////////////////////////////////////////////////////////////
// CRC & DISPATCH
///////////////////////////////////////////////////////////////

uint8_t SerialHandler::_computeCRC(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

void SerialHandler::_sendPacket(uint8_t msg_id, uint8_t dir, const uint8_t* payload, uint8_t len) {

    uint8_t frame[HEADER_SIZE_BYTES + MAX_PAYLOAD_BYTES];
    frame[0] = SOF_BYTE;
    frame[1] = SERIAL_PROTOCOL_VERSION;
    frame[2] = msg_id;
    frame[3] = dir;
    frame[4] = len;

    if (payload && len > 0) {
        memcpy(frame + HEADER_SIZE_BYTES, payload, len);
    }

    uint8_t crc = _computeCRC(frame, HEADER_SIZE_BYTES + len);
    Serial.write(frame, HEADER_SIZE_BYTES + len);
    Serial.write(crc);
}

void SerialHandler::_sendACK(uint8_t ack_msg_id) {
    uint8_t payload[1] = { ack_msg_id };
    _sendPacket(MSG_ACK, DIR_MCU_TO_PC, payload, 1);
}

void SerialHandler::_sendNACK(uint8_t nack_msg_id, NackError error) {
    uint8_t payload[2] = { nack_msg_id, (uint8_t)error };
    _sendPacket(MSG_NACK, DIR_MCU_TO_PC, payload, 2);
}

///////////////////////////////////////////////////////////////
// PUBLIC API
///////////////////////////////////////////////////////////////

void SerialHandler::sendParameter(uint8_t parameter_id, float value) {
    FloatBytes fb;
    fb.f = value;

    uint8_t payload[5] = { parameter_id, fb.b[0], fb.b[1], fb.b[2], fb.b[3] };

    _sendPacket(MSG_CMD_GET, DIR_MCU_TO_PC, payload, 5);
}

void SerialHandler::sendStatus(uint8_t status_code) {
    uint8_t payload[1] = { status_code };
    _sendPacket(MSG_STATUS, DIR_MCU_TO_PC, payload, 1);
}

void SerialHandler::sendHeartbeat() {
    _sendPacket(MSG_HEARTBEAT, DIR_MCU_TO_PC, nullptr, 0);
}

void SerialHandler::_onCmdSet(const Packet& pkt) {
    if (pkt.len < 5) { _sendNACK(pkt.msg_id, ERR_BAD_LEN); return; }
    if (!_setCb)      { _sendNACK(pkt.msg_id, ERR_UNKNOWN_MSG); return; }

    uint8_t parameter_id = pkt.payload[0];

    FloatBytes fb;
    fb.b[0] = pkt.payload[1];
    fb.b[1] = pkt.payload[2];
    fb.b[2] = pkt.payload[3];
    fb.b[3] = pkt.payload[4];

    _setCb(parameter_id, fb.f);
    _sendACK(pkt.msg_id);
}

void SerialHandler::_onCmdGet(const Packet& pkt) {
    if (pkt.len < 1) { _sendNACK(pkt.msg_id, ERR_BAD_LEN); return; }
    if (!_getCb)     { _sendNACK(pkt.msg_id, ERR_UNKNOWN_MSG); return; }

    uint8_t parameter_id = pkt.payload[0];
    sendParameter(parameter_id, _getCb(parameter_id));
}