/**
 * @file FlyskyIBUS.cpp
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include "FlyskyIBUS.h"

// Constructor: Initializes the IBUS receiver with specified UART, RX, and TX pins.
FlyskyIBUS::FlyskyIBUS(HardwareSerial &uart, uint8_t rxPin, uint8_t txPin) : _uart(&uart),
                                                                             _rxPin(rxPin),
                                                                             _txPin(txPin),
                                                                             _frame_position(IBUS_FRAME_START),
                                                                             _channels{IBUS_FAILSAFE_CHANNEL_VALUE},
                                                                             _isFailsafeActive(true)
{
}

// Initializes the UART communication for IBUS.
void FlyskyIBUS::begin()
{
    // Configure UART with specified baud rate, 8 data bits, no parity, 1 stop bit, and pins.
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, _rxPin, _txPin);
}

// Retrieves the value for a specific channel (0-based).
// Automatically processes available UART data before returning the channel value.
uint16_t FlyskyIBUS::getChannel(const uint8_t channel_nr)
{
    // Validate channel number to prevent out-of-bounds access.
    if (channel_nr >= IBUS_MAX_CHANNELS)
    {
        return IBUS_FAILSAFE_CHANNEL_VALUE;
    }

    // Process all currently available UART bytes to update channel data.
    while (_uart->available())
    {
        _processByte(_uart->read());
    }

    return _channels[channel_nr];
}

// Retrieves all channel values as a `std::array`.
// Automatically processes available UART data before returning the array.
std::array<uint16_t, FlyskyIBUS::IBUS_MAX_CHANNELS> FlyskyIBUS::getChannels()
{
    // Process any pending UART data to ensure channels are up-to-date.
    while (_uart->available())
    {
        _processByte(_uart->read());
    }

    // Create a `std::array` and copy the internal channel values.
    std::array<uint16_t, FlyskyIBUS::IBUS_MAX_CHANNELS> allChannels;

    for (auto i = 0; i < FlyskyIBUS::IBUS_MAX_CHANNELS; ++i)
    {
        allChannels[i] = _channels[i];
    }

    return allChannels;
}

// Processes a single byte from the UART stream to construct an IBUS frame.
void FlyskyIBUS::_processByte(uint8_t byte)
{
    // If at the start of a frame, wait for the first header byte (IBUS_HEADER_BYTE0).
    if (_frame_position == IBUS_FRAME_START)
    {
        if (byte == IBUS_HEADER_BYTE0)
        {
            _frame_buffer[_frame_position++] = byte;
        }
        return;
    }

    // If after the first header byte, wait for the second header byte (IBUS_HEADER_BYTE1).
    if (_frame_position == IBUS_HEADER_START)
    {
        if (byte == IBUS_HEADER_BYTE1)
        {
            _frame_buffer[_frame_position++] = byte;
        }
        else
        {
            // If the second byte is not the header, reset frame reception.
            _frame_position = IBUS_FRAME_START;
        }
        return;
    }

    // Fill the frame buffer with payload and checksum bytes.
    _frame_buffer[_frame_position++] = byte;

    // If the frame is complete, process it.
    if (_frame_position >= IBUS_FRAME_LENGTH)
    {
        _processFrame();
        // Reset frame position to start waiting for the next frame.
        _frame_position = IBUS_FRAME_START;
    }
}

// Processes a complete received IBUS frame, validates the checksum, and decodes channels.
void FlyskyIBUS::_processFrame()
{
    // Extract the received checksum from the end of the buffer.
    const uint16_t checksum = (_frame_buffer[IBUS_CRC_HIGH_BYTE_INDEX] << IBUS_BIT_SHIFT) | _frame_buffer[IBUS_CRC_LOW_BYTE_INDEX];

    // Calculate the checksum for all bytes in the frame (excluding the checksum bytes themselves).
    auto sum = 0;

    for (auto i = 0; i < IBUS_CRC_LOW_BYTE_INDEX; ++i)
    {
        sum += _frame_buffer[i];
    }
    const uint16_t calculated_checksum = IBUS_CRC_MINUEND - sum;

    // Compare the received checksum with the calculated checksum.
    if (checksum == calculated_checksum)
    {
        // If checksums match, the frame is valid, so decode the channel values.
        _decode_channels();
    }
}

// Decodes raw byte values from the frame buffer into 16-bit channel values.
void FlyskyIBUS::_decode_channels()
{
    for (auto i = 0; i < IBUS_MAX_CHANNELS; ++i)
    {
        // Each channel value is composed of two bytes in the frame.
        const auto channel_byte_index = PAYLOAD_HIGHBYTE + (i << 1);

        // Read low and high bytes for the current channel.
        uint8_t lowByte = _frame_buffer[channel_byte_index];
        uint8_t highByte = _frame_buffer[channel_byte_index + 1];

        // Combine bytes to form the 16-bit channel value (little-endian).
        uint16_t value = (highByte << IBUS_BIT_SHIFT) | lowByte;

        // Store the decoded channel value.
        _channels[i] = value;
    }

    // After decoding, perform a failsafe check.
    _failsafe_check();
}

// Checks if failsafe conditions are met based on the sum of the first few channels.
void FlyskyIBUS::_failsafe_check()
{
    auto sum = 0;

    for (auto i = 0; i < IBUS_FAILSAFE_CHANNEL_COUNT; ++i)
        sum += _channels[i];

    // Failsafe is active if the sum of the first IBUS_FAILSAFE_CHANNEL_COUNT channels is below a threshold.
    _isFailsafeActive = (sum <= IBUS_FAILSAVE_SUM);
}