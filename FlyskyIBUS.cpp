/**
 * @file FlyskyIBUS.cpp
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#pragma once

#include "FlyskyIBUS.h"

// Initialize IBUS in "on the fly"
FlyskyIBUS::FlyskyIBUS(HardwareSerial &uart, uint8_t rxPin) : _uart(&uart),
                                                              _rxPin(rxPin),
                                                              _frame_position(0),
                                                              _frameStarted(false),
                                                              _channelCount(0),
                                                              _frame_buffer{uint8_t(IBUS_DEFAULT_VALUE)}
{
}

// IBUS receiver "install and forget"
bool FlyskyIBUS::begin()
{
    // Setup IBUS UART
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, _rxPin);

    // Tricky UART Interrupt
    // Install UART interrupt triggering IBUS handle
    _uart->onReceive([this]()
                     { this->_ibus_handle(); });

    return IBUS_OK;
}

// Returns received value for given channel
uint16_t FlyskyIBUS::getChannel(const uint8_t channel_nr)
{
    // Simple input filter workround
    if (channel_nr < 1 || channel_nr > IBUS_MAX_CHANNELS)
        return IBUS_DEFAULT_VALUE;

    // Array - Human Transcoder
    return _channels[channel_nr - 1];
}

// Reading data bytewise into frame generator
void IRAM_ATTR FlyskyIBUS::_ibus_handle()
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
    // Find IBUS frame start marking
    if (!_frameStarted && byte == IBUS_HEADER_BYTE0)
    {
        _frameStarted = true;
        _frame_position = 0;

        _frame_buffer[_frame_position++] = byte;
    }
    // Fill up the frame buffer
    else if (_frameStarted)
    {
        // Receiving frame bytes
        if (_frame_position < IBUS_FRAME_LENGTH)
        {
            _frame_buffer[_frame_position++] = byte;
        }

        // End of IBUS frame reached
        if (_frame_position >= IBUS_FRAME_LENGTH)
        {
            // TODO: use reference and return values
            _decode_channels();

            // Dirty restart frame container
            _frameStarted = false;
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

        _channels[i] = (highByte << 8) | lowByte;
    }
}
