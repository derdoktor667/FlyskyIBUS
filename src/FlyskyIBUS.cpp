/**
 * @file FlyskyIBUS.cpp
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include "FlyskyIBUS.h"

// Initialize IBUS in "on the fly"
FlyskyIBUS::FlyskyIBUS(HardwareSerial &uart, uint8_t rxPin, uint8_t txPin) : _uart(&uart),
                                                              _rxPin(rxPin),
                                                              _txPin(txPin),
                                                              _frame_position(0),
                                                              _channelCount(0),
                                                              _frame_buffer{uint8_t(IBUS_DEFAULT_VALUE)},
                                                              _lastReadTime(0),
                                                              _failsafe_flag(false)
{
    for (size_t i = 0; i < IBUS_MAX_CHANNELS; ++i)
    {
        _channels[i] = IBUS_DEFAULT_VALUE;
    }
}

// IBUS receiver "install and forget"
bool FlyskyIBUS::begin()
{
    // Setup IBUS UART
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, _rxPin, _txPin);
    return IBUS_OK;
}

// Returns received value for given channel
uint16_t FlyskyIBUS::getChannel(const uint8_t channel_nr) const
{
    return getRawChannel(channel_nr);
}

// Returns raw received value for given channel
uint16_t FlyskyIBUS::getRawChannel(const uint8_t channel_nr) const
{
    // Simple input filter workround
    if (channel_nr < 1 || channel_nr > IBUS_MAX_CHANNELS)
        return IBUS_DEFAULT_VALUE;

    // Array - Human Transcoder
    return _channels[channel_nr - 1];
}

// Check for failsafe condition
bool FlyskyIBUS::hasFailsafe() const
{
    uint32_t lastReadTimeCopy;
    portENTER_CRITICAL_ISR(&_mux);
    lastReadTimeCopy = _lastReadTime;
    portEXIT_CRITICAL_ISR(&_mux);

    // Failsafe is active if the timeout is exceeded or the failsafe flag is set
    return ((millis() - lastReadTimeCopy) > IBUS_SIGNAL_TIMEOUT) || _failsafe_flag;
}

// Reading data bytewise into frame generator
void FlyskyIBUS::read()
{
    while (_uart->available())
    {
        // Feed the generator
        uint8_t received_byte = _uart->read();
        _generateFrame(received_byte);
    }
}

// Generates the actual IBUS frame and start processing
void FlyskyIBUS::_generateFrame(uint8_t byte)
{
    if (_frame_position == 0)
    { // Waiting for the first header byte
        if (byte == IBUS_HEADER_BYTE0)
        {
            _frame_buffer[_frame_position++] = byte;
        }
    }
    else if (_frame_position == 1)
    {
        if (byte == IBUS_HEADER_BYTE1)
        {
            _frame_buffer[_frame_position++] = byte;
        }
        else
        {
            // Reset if the second byte is not correct
            _frame_position = 0;
        }
    }
    else
    {
        if (_frame_position < IBUS_FRAME_LENGTH)
        {
            _frame_buffer[_frame_position++] = byte;
        }

        if (_frame_position >= IBUS_FRAME_LENGTH)
        {
            uint16_t checksum = (_frame_buffer[31] << 8) | _frame_buffer[30];
            uint16_t calculated_checksum = 0xFFFF;
            for (int i = 0; i < (IBUS_FRAME_LENGTH - IBUS_CRC_LENGTH); i++)
            {
                calculated_checksum -= _frame_buffer[i];
            }

            if (checksum == calculated_checksum)
            {
                _decode_channels();
            }
            _frame_position = 0;
        }
    }
}

// Reading and decoding buffer values into buffer array
void FlyskyIBUS::_decode_channels()
{
    // Calculate number of channels from payload length
    _channelCount = (_frame_buffer[0] - IBUS_HEADER_LENGTH - IBUS_CRC_LENGTH) >> 1;

    for (size_t i = 0; i < IBUS_MAX_CHANNELS && i < _channelCount; ++i)
    {
        // Byte to channel merging
        uint8_t lowByte = _frame_buffer[PAYLOAD_HIGHBYTE + i * 2];
        uint8_t highByte = _frame_buffer[PAYLOAD_LOWBYTE + i * 2];
        uint16_t value = (highByte << 8) | lowByte;

        _channels[i] = value;
    }

    // Check for specific failsafe condition: first four channels are 988
    bool channel_failsafe_active = true;
    for (size_t i = 0; i < IBUS_FAILSAFE_CHANNEL_COUNT; ++i)
    {
        if (_channels[i] != IBUS_FAILSAFE_CHANNEL_VALUE)
        {
            channel_failsafe_active = false;
            break;
        }
    }
    _failsafe_flag = channel_failsafe_active;

    // A valid frame has been received, reset the timer
    _lastReadTime = millis();
}
