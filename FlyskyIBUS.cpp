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
                                                                             _channels{IBUS_FAILSAFE_CHANNEL_VALUE},
                                                                             _isFailsafeActive(true)
{
}

// IBUS receiver "install and forget"
void FlyskyIBUS::begin()
{
    // Setup IBUS UART
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, _rxPin, _txPin);
}

// Returns received value for given channel
uint16_t FlyskyIBUS::getChannel(const uint8_t channel_nr)
{
    // Check for valid channel number
    if (channel_nr >= IBUS_MAX_CHANNELS)
    {
        return IBUS_FAILSAFE_CHANNEL_VALUE;
    }

    while (_uart->available())
    {
        // Feed the generator with UART input
        _generateFrame(_uart->read());
    }

    return _channels[channel_nr];
}

// Generates the actual IBUS frame and start processing
void FlyskyIBUS::_generateFrame(uint8_t byte)
{
    if (_frame_position == IBUS_FRAME_START)
    { // Waiting for the first header byte
        if (byte == IBUS_HEADER_BYTE0)
        {
            _frame_buffer[_frame_position++] = byte;
        }
    }
    else if (_frame_position == IBUS_HEADER_START)
    {
        if (byte == IBUS_HEADER_BYTE1)
        {
            _frame_buffer[_frame_position++] = byte;
        }
        else
        {
            // Reset if the second byte is not correct
            _frame_position = IBUS_FRAME_START;
        }
    }
    else
    {
        if (_frame_position < IBUS_FRAME_LENGTH)
        {
            _frame_buffer[_frame_position++] = byte;
        }

        // Check for valid frame by checksum
        if (_frame_position >= IBUS_FRAME_LENGTH)
        {
            // Received checksum
            uint16_t checksum = (_frame_buffer[IBUS_CHECKSUM_HIGH_BYTE_INDEX] << IBUS_CRC_SHIFT) | _frame_buffer[IBUS_CHECKSUM_LOW_BYTE_INDEX];

            // Calculate checksum for received bytes
            uint16_t calculated_checksum = IBUS_CRC_MINUEND;

            // Sum up received bytes after header bytes
            uint16_t sum = IBUS_FRAME_START;

            for (size_t i = 0; i < IBUS_CHECKSUM_LOW_BYTE_INDEX; ++i)
            {
                sum += _frame_buffer[i];
            }

            calculated_checksum -= sum;

            // Compare the checksums
            if (checksum == calculated_checksum)
            {
                // All good, start decoding
                _decode_channels();
            }

            // Reset the frame
            _frame_position = IBUS_FRAME_START;
        }
    }
}

// Reading and decoding buffer values into buffer array
void FlyskyIBUS::_decode_channels()
{
    for (size_t i = 0; i < IBUS_MAX_CHANNELS; ++i)
    {
        // Byte to channel merging
        uint8_t lowByte = _frame_buffer[PAYLOAD_HIGHBYTE + (i << 1)];
        uint8_t highByte = _frame_buffer[PAYLOAD_LOWBYTE + (i << 1)];

        // Reorder values
        uint16_t value = (highByte << IBUS_BYTE_SHIFT) | lowByte;

        // Fill the array with the decoded value
        _channels[i] = value;
    }

    // Check channels for Failsafe
    _failsafe_check();
}

void FlyskyIBUS::_failsafe_check()
{
    uint16_t sum = 0;

    for (size_t i = 0; i < IBUS_FAILSAFE_CHANNEL_COUNT; ++i)
        sum += _channels[i];

    _isFailsafeActive = (sum <= IBUS_FAILSAVE_SUM);
}
