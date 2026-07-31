#include "SerialManager.h"
#include "PowderDrumProtocol.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::begin(uint32_t baudRate)
{
    Serial.begin(baudRate);
    _state = SerialState::IDLE;
    _bytesRead = 0;
    _lastByteTime = millis();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::update()
{
    // Mid-packet timeout guard — bail back to IDLE if a frame stalls
    if (_state != SerialState::IDLE && (millis() - _lastByteTime > TIMEOUT_MS))
    {
        _state = SerialState::IDLE;
        _bytesRead = 0;
        return;
    }

    // Main state machine
    switch(_state)
    {
        case SerialState::IDLE:         _handleIdle();        break;
        case SerialState::READ_HEADER:  _handleReadHeader();  break;
        case SerialState::READ_PAYLOAD: _handleReadPayload(); break;
        case SerialState::VALIDATE:     _handleValidate();    break;
        case SerialState::DISPATCH:     _handleDispatch();    break;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_handleIdle()
{
    while(Serial.available())
    {
        if(Serial.read() != SOF_BYTE) continue;

        _rxBuffer[0] = SOF_BYTE;
        _bytesRead = 1;
        _lastByteTime = millis();
        _state = SerialState::READ_HEADER;
        return;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_handleReadHeader()
{
    while(Serial.available() && _bytesRead < HEADER_SIZE_BYTES)
    {
        _rxBuffer[_bytesRead++] = Serial.read();
        _lastByteTime = millis();
    }

    if(_bytesRead < HEADER_SIZE_BYTES) return;

    _rxPacket.version   = _rxBuffer[1];
    _rxPacket.msgID     = _rxBuffer[2];
    _rxPacket.direction = _rxBuffer[3];
    _rxPacket.length    = _rxBuffer[4];

    // Reject mismatched version immediately
    if (_rxPacket.version != SERIAL_PROTOCOL_VERSION)
    {
        _sendNACK(_rxPacket.msgID, ERR_VERSION_MISMATCH);
        _state = SerialState::IDLE;
        _bytesRead = 0;
        return;
    }

    // Reject anything not addressed to us
    if (_rxPacket.direction != DIR_PC_TO_MCU)
    {
        _sendNACK(_rxPacket.msgID, ERR_BAD_DIRECTION);
        _state = SerialState::IDLE;
        _bytesRead = 0;
        return;
    }

    // Reject an implausible length before we commit to waiting for it
    if (_rxPacket.length > MAX_PAYLOAD_BYTES)
    {
        _sendNACK(_rxPacket.msgID, ERR_BAD_LEN);
        _state = SerialState::IDLE;
        _bytesRead = 0;
        return;
    }

    _state = SerialState::READ_PAYLOAD;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_handleReadPayload()
{
    uint16_t expectedBytes = HEADER_SIZE_BYTES + _rxPacket.length + 1; // + CRC byte

    while(Serial.available() && _bytesRead < expectedBytes)
    {
        _rxBuffer[_bytesRead++] = Serial.read();
        _lastByteTime = millis();
    }

    if(_bytesRead < expectedBytes) return;

    _state = SerialState::VALIDATE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_handleValidate()
{
    uint8_t receivedCRC = _rxBuffer[HEADER_SIZE_BYTES + _rxPacket.length];
    uint8_t computedCRC = _computeCRC(_rxBuffer, HEADER_SIZE_BYTES + _rxPacket.length);

    if (receivedCRC != computedCRC)
    {
        _sendNACK(_rxPacket.msgID, ERR_BAD_CRC);
        _state = SerialState::IDLE;
        _bytesRead = 0;
        return;
    }

    _rxPacket.crc = receivedCRC;
    memcpy(_rxPacket.payload, &_rxBuffer[HEADER_SIZE_BYTES], _rxPacket.length);
    _state = SerialState::DISPATCH;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_handleDispatch()
{
    switch (_rxPacket.msgID)
    {
        case MSG_CMD_SET:   _onCmdSet(_rxPacket);  break;
        case MSG_CMD_GET:   _onCmdGet(_rxPacket);  break;
        case MSG_HEARTBEAT: sendHeartbeat();       break;
        default:
            _sendNACK(_rxPacket.msgID, ERR_UNKNOWN_MSG);
            break;
    }

    _state = SerialState::IDLE;
    _bytesRead = 0;
    _rxPacket = Packet{};
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_onCmdSet(const Packet& packet)
{
    if (packet.length < 5) { _sendNACK(packet.msgID, ERR_BAD_LEN); return; }

    uint8_t parameterID = packet.payload[0];

    if (!PowderDrumProtocol::exists(parameterID))     { _sendNACK(packet.msgID, ERR_UNKNOWN_PARAM); return; }
    if (!PowderDrumProtocol::isWritable(parameterID)) { _sendNACK(packet.msgID, ERR_READ_ONLY);      return; }
    if (!_setCallback)                                { _sendNACK(packet.msgID, ERR_UNKNOWN_MSG);    return; }

    FloatBytes fb;
    fb.b[0] = packet.payload[1];
    fb.b[1] = packet.payload[2];
    fb.b[2] = packet.payload[3];
    fb.b[3] = packet.payload[4];

    _setCallback(parameterID, fb.f);
    _sendACK(packet.msgID);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_onCmdGet(const Packet& packet)
{
    if (packet.length < 1) { _sendNACK(packet.msgID, ERR_BAD_LEN); return; }

    uint8_t parameterID = packet.payload[0];

    if (!PowderDrumProtocol::exists(parameterID))     { _sendNACK(packet.msgID, ERR_UNKNOWN_PARAM); return; }
    if (!PowderDrumProtocol::isReadable(parameterID)) { _sendNACK(packet.msgID, ERR_WRITE_ONLY);     return; }
    if (!_getCallback)                                { _sendNACK(packet.msgID, ERR_UNKNOWN_MSG);    return; }

    sendParameter(parameterID, _getCallback(parameterID));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t SerialManager::_computeCRC(const uint8_t* data, uint8_t length)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < length; i++) crc ^= data[i];
    return crc;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_sendPacket(uint8_t msgID, uint8_t direction, const uint8_t* payload, uint8_t length)
{
    uint8_t frame[HEADER_SIZE_BYTES + MAX_PAYLOAD_BYTES];
    frame[0] = SOF_BYTE;
    frame[1] = SERIAL_PROTOCOL_VERSION;
    frame[2] = msgID;
    frame[3] = direction;
    frame[4] = length;

    if (payload && length > 0) memcpy(frame + HEADER_SIZE_BYTES, payload, length);

    uint8_t crc = _computeCRC(frame, HEADER_SIZE_BYTES + length);
    Serial.write(frame, HEADER_SIZE_BYTES + length);
    Serial.write(crc);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_sendACK(uint8_t acknowledgedMessage)
{
    uint8_t payload[1] = { acknowledgedMessage };
    _sendPacket(MSG_ACK, DIR_MCU_TO_PC, payload, 1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::_sendNACK(uint8_t rejectedMessage, NackError error)
{
    uint8_t payload[2] = { rejectedMessage, (uint8_t)error };
    _sendPacket(MSG_NACK, DIR_MCU_TO_PC, payload, 2);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::sendParameter(uint8_t parameterID, float value)
{
    FloatBytes fb; fb.f = value;
    uint8_t payload[5] = { parameterID, fb.b[0], fb.b[1], fb.b[2], fb.b[3] };
    _sendPacket(MSG_CMD_GET, DIR_MCU_TO_PC, payload, 5);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::sendStatus(uint8_t statusCode)
{
    uint8_t payload[1] = { statusCode };
    _sendPacket(MSG_STATUS, DIR_MCU_TO_PC, payload, 1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::sendHeartbeat() { _sendPacket(MSG_HEARTBEAT, DIR_MCU_TO_PC, nullptr, 0); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void SerialManager::onSet(SetCallback callback) { _setCallback = callback; }
void SerialManager::onGet(GetCallback callback) { _getCallback = callback; }