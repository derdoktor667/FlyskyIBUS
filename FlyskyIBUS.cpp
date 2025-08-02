/**
 * @file FlyskyIBUS.cpp - Native UART Interrupt Version
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include <Arduino.h>
#include <FlyskyIBUS.h>

//
FlyskyIBUS::FlyskyIBUS()
    : _uart(nullptr), _rxPin(0), _frame_position(0), _frameStarted(false), _lastFrameTime(0), _channelCount(0), _frame_buffer()
{
}

//
bool FlyskyIBUS::begin(HardwareSerial &uart, uint8_t rxPin)
{
    _uart = &uart;
    _rxPin = rxPin;

    // Setup UART for IBUS
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, _rxPin, -1);

    // Tricky UART Interrupt 
    _uart->onReceive([this]()
                     { this->onSerialReceive(); });

    // Prepare Channel Container
    initChannels();

    return 0;
}

//
void FlyskyIBUS::onSerialReceive()
{
    while (_uart->available())
    {
        uint8_t byte = _uart->read();

        //
        processByte(byte);
    }
}

//
void FlyskyIBUS::processByte(uint8_t byte)
{
    //
    if (!_frameStarted && byte == IBUS_HEADER_BYTE0)
    {
        _frameStarted = true;
        _frame_position = 0;

        _frame_buffer[_frame_position++] = byte;
    }
    //
    else if (_frameStarted)
    {
        //
        if (_frame_position < IBUS_FRAME_LENGTH)
        {
            _frame_buffer[_frame_position++] = byte;
        }

        //
        if (_frame_position >= IBUS_FRAME_LENGTH)
        {
            //
            decodeChannels();

            //
            _frameStarted = false;
            _frame_position = 0;
            _lastFrameTime = millis();
        }
    }
}

//
void FlyskyIBUS::decodeChannels()
{
    // Calculate number of channels from payload length
    _channelCount = (_frame_buffer[0] - IBUS_HEADER_LENGTH - IBUS_CRC_LENGTH) >> 1;

    for (size_t i = 0; i < IBUS_MAX_CHANNELS && i < _channelCount; ++i)
    {
        // Byte to channel merging
        uint8_t lowByte = _frame_buffer[PAYLOAD_HIGHBYTE + i * 2];
        uint8_t highByte = _frame_buffer[PAYLOAD_LOWBYTE + i * 2];

        _channels[i] = (highByte << 8) | lowByte;
    }
}

//
uint16_t FlyskyIBUS::readChannel(uint8_t channel)
{
    if (channel < 1 || channel > IBUS_MAX_CHANNELS)
        return IBUS_DEFAULT_VALUE;
    
    //
    return _channels[channel - 1];
}

//
uint8_t FlyskyIBUS::getChannelCount()
{
    return _channelCount;
}

//
bool FlyskyIBUS::isConnected()
{
    return (millis() - _lastFrameTime) < IBUS_SIGNAL_TIMEOUT;
}

//
void FlyskyIBUS::initChannels()
{
    // Resets channels to default value
    for (size_t i = 0; i < IBUS_MAX_CHANNELS; i++)
    {
        _channels[i] = IBUS_DEFAULT_VALUE;
    }
}

//
void FlyskyIBUS::resetBuffer()
{
    _frame_position = 0;
    _frameStarted = false;
}
