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
        // Feed the processor with UART input
        _processByte(_uart->read());
    }

    return _channels[channel_nr];
}

// Processes a single byte from the UART stream to build a frame
void FlyskyIBUS::_processByte(uint8_t byte)
{
    // State 0: Waiting for header byte 1 (0x20)
    if (_frame_position == 0)
    {
        if (byte == IBUS_HEADER_BYTE0)
        {
            _frame_buffer[_frame_position++] = byte;
        }
        return;
    }

    // State 1: Waiting for header byte 2 (0x40)
    if (_frame_position == 1)
    {
        if (byte == IBUS_HEADER_BYTE1)
        {
            _frame_buffer[_frame_position++] = byte;
        }
        else
        {
            // Second byte is not the header, reset.
            _frame_position = 0;
        }
        return;
    }

    // States 2 to 31: Filling the payload and checksum
    _frame_buffer[_frame_position++] = byte;

    // State 32: Frame is full, process it.
    if (_frame_position >= IBUS_FRAME_LENGTH)
    {
        _processFrame();
        // Reset for the next frame
        _frame_position = 0;
    }
}

// Processes a complete frame, validating checksum and decoding channels
void FlyskyIBUS::_processFrame()
{
    // Received checksum from the buffer
    const uint16_t checksum = (_frame_buffer[IBUS_CHECKSUM_HIGH_BYTE_INDEX] << IBUS_BYTE_SHIFT) | _frame_buffer[IBUS_CHECKSUM_LOW_BYTE_INDEX];

    // Calculate checksum for the received bytes
    uint16_t sum = 0;
    for (size_t i = 0; i < IBUS_CHECKSUM_LOW_BYTE_INDEX; ++i)
    {
        sum += _frame_buffer[i];
    }
    const uint16_t calculated_checksum = IBUS_CRC_MINUEND - sum;

    // Compare checksums
    if (checksum == calculated_checksum)
    {
        // Checksum is valid, decode the channels
        _decode_channels();
    }
}

// Reading and decoding buffer values into buffer array
void FlyskyIBUS::_decode_channels()
{
    for (size_t i = 0; i < IBUS_MAX_CHANNELS; ++i)
    {
        // Each channel is represented by two bytes in the frame
        const size_t channel_byte_index = PAYLOAD_HIGHBYTE + (i << 1);

        // Byte to channel merging
        uint8_t lowByte = _frame_buffer[channel_byte_index];
        uint8_t highByte = _frame_buffer[channel_byte_index + 1];

        // Reorder values
        uint16_t value = (highByte << IBUS_BYTE_SHIFT) | lowByte;

        // Fill the array with the decoded value
        _channels[i] = value;
    }

    // Check channels for Failsafe
    _failsafe_check();
}

// The failsafe is active when the sum of the first 4 channels is below a certain threshold.
// This is a safety feature of the IBUS protocol.
void FlyskyIBUS::_failsafe_check()
{
    uint16_t sum = 0;

    for (size_t i = 0; i < IBUS_FAILSAFE_CHANNEL_COUNT; ++i)
        sum += _channels[i];

    _isFailsafeActive = (sum <= IBUS_FAILSAVE_SUM);
}
