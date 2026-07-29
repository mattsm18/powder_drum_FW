#include "SerialManager.h"

void SerialHandler::begin(uint32_t baudRate)
{
    Serial.begin(baudRate);
    _state = SerialState::IDLE;

    _bytesRead = 0;
    _lastByteTime = millis();

}

void SerialHandler::update()
{
    switch(_state)
    {
        case SerialState::IDLE:             _handleIdle();          break;
        case SerialState::READ_HEADER:      _handleReadHeader();    break;
        case SerialState::READ_PAYLOAD:     _handleReadPayload();   break;
        case SerialState::VALIDATE:         _handleValidate();      break;
        case SerialState::DISPATCH:         _handleDispatch();      break;                
    }
}

// Wait until SOF_BYTE recieved, store and change to next state
void SerialHandler::_handleIdle()
{
    while(Serial.available())
    {
        if(Serial.read() != SOF_BYTE) continue;

        _rxBuffer[0] = SOF_BYTE;
        _bytesRead = 1;
        _lastByteTime = millis();
        _state = SerialState::READ_HEADER; // NEXT STATE

        return;
    }
}

// Read header until full
void SerialHandler::_handleReadHeader()
{
    while(Serial.available() && _bytesRead < HEADER_SIZE_BYTES)
    {
        _rxBuffer[_bytesRead++] = Serial.read();
        _lastByteTime = millis();
    }

    if(_bytesRead < HEADER_SIZE_BYTES) return;

    _state = SerialState::READ_PAYLOAD; // NEXT STATE
}

// Read payload until full
void SerialHandler::_handleReadPayload()
{
    uint8_t payloadLength = _rxBuffer[4];
    uint16_t expectedBytes = HEADER_SIZE_BYTES + payloadLength + 1;

    while(Serial.available() && _bytesRead < expectedBytes)
    {
        _rxBuffer[_bytesRead++] = Serial.read();
        _lastByteTime = millis();
    }

    if(_bytesRead < expectedBytes) return; 

    _state = SerialState::VALIDATE; // NEXT STATE
}

