/**
 * @file FlyskyIBUS.cpp
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include <Arduino.h>
#include <FlyskyIBUS.h>
#include <HardwareSerial.h>
#include <portmacro.h>

// Static instance for interrupt handling
FlyskyIBUS *FlyskyIBUS::_instance = nullptr;

// Constructor - initialize all variables to safe defaults
FlyskyIBUS::FlyskyIBUS()
{
    _uart = nullptr;
    _framePosition = 0;
    _frameInProgress = false;
    _bufferChannelCount = 0;

    // Set static instance for interrupt handling
    _instance = this;

    // Prepare Containers
    initChannels();
    resetIbusBuffer();
}

// Initialize with custom RX pin (TX disabled)
void FlyskyIBUS::begin(HardwareSerial &uart = Serial2, uint8_t rxPin = GPIO_NUM_16)
{
    _uart = &uart;
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, rxPin, -1);
    installIbusInterrupt();
}

// Read specific channel value (1-based channel numbering)
uint16_t FlyskyIBUS::readChannel(uint8_t channelNumber)
{
    if (channelNumber >= 1 && channelNumber <= IBUS_MAX_CHANNELS)
    {
        return _channels[channelNumber - 1]; // Convert to 0-based array index
    }

    return IBUS_DEFAULT_VALUE;
}

// Copy all channel values to user-provided array
void FlyskyIBUS::readAllChannels(uint16_t *channelArray)
{
    if (channelArray == nullptr)
    {
        return;
    }

    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++)
    {
        channelArray[i] = _channels[i];
    }
}

// Setup UART interrupt for automatic data reception
void FlyskyIBUS::installIbusInterrupt()
{
    if (!_uart)
        return;

    _uart->onReceive(ibusHandler);
}

// Interrupt processing
void IRAM_ATTR FlyskyIBUS::ibusHandler()
{
    if (_instance && _instance->_uart && _instance->_uart->available())
    {
        while (_instance->_uart->available())
        {
            // Interrupt triggert
            uint8_t byte = _instance->_uart->read();
            _instance->processIbusBytes(byte);
        }
    }
}

// Calculate in sync with interrupt
void FlyskyIBUS::processIbusBytes(uint8_t byte)
{
    // Check for frame start sequence
    if (!_frameInProgress)
    {
        if (_framePosition == 0 && byte == IBUS_HEADER_BYTE1)
        {
            _frameBuffer[_framePosition++] = byte;
        }
        else if (_framePosition == 1 && byte == IBUS_HEADER_BYTE2)
        {
            _frameBuffer[_framePosition++] = byte;
            _frameInProgress = true;
        }
        else
        {
            _framePosition = 0;
            if (byte == IBUS_HEADER_BYTE1)
            {
                _frameBuffer[_framePosition++] = byte;
            }
        }
        return;
    }

    // Frame in progress - collect data
    if (_framePosition < IBUS_FRAME_LENGTH)
    {
        _frameBuffer[_framePosition++] = byte;

        // Check if frame is complete
        if (_framePosition == IBUS_FRAME_LENGTH)
        {
            // Validate checksum
            uint16_t calculatedChecksum = 0xFFFF;
            for (uint8_t i = 0; i < (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE); i++)
            {
                calculatedChecksum -= _frameBuffer[i];
            }

            uint8_t checksumLowByte = _frameBuffer[IBUS_FRAME_LENGTH - 2];
            uint8_t checksumHighByte = _frameBuffer[IBUS_FRAME_LENGTH - 1];
            uint16_t frameChecksum = (checksumHighByte << 8) | checksumLowByte;

            if (calculatedChecksum == frameChecksum)
            {
                decodeIbusChannels();
            }

            // Reset frame
            _framePosition = 0;
            _frameInProgress = false;
        }
    }
    else
    {
        // Frame overflow
        _framePosition = 0;
        _frameInProgress = false;
    }
}

// Interrupt-safe channel decoding
void FlyskyIBUS::decodeIbusChannels()
{
    _bufferChannelCount = 0;
    uint8_t channelDataStart = IBUS_HEADER_SIZE;

    // Decode channels in buffer
    for (uint8_t channelIndex = 0; channelIndex < IBUS_MAX_CHANNELS; channelIndex++)
    {
        uint8_t lowBytePosition = channelDataStart + (channelIndex * IBUS_BYTES_PER_CHANNEL);
        uint8_t highBytePosition = lowBytePosition + 1;

        // Check if we have enough data for this channel
        if (highBytePosition >= (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE))
        {
            break;
        }

        // Extract channel value (little-endian)
        uint8_t lowByte = _frameBuffer[lowBytePosition];
        uint8_t highByte = _frameBuffer[highBytePosition];
        uint16_t channelValue = (highByte << 8) | lowByte;

        // Validate channel value
        if (channelValue >= IBUS_MIN_VALUE && channelValue <= IBUS_MAX_VALUE)
        {
            _channelBuffer[channelIndex] = channelValue;
            _bufferChannelCount++;
        }
        else
        {
            // Invalid channel value - stop processing
            break;
        }
    }
}

// Calculate calculate CRC
uint16_t FlyskyIBUS::calculateCRC()
{
    uint16_t checksum = 0xFFFF;

    // Calculate checksum for all bytes except the checksum bytes at the end
    for (uint8_t i = 0; i < (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE); i++)
    {
        checksum -= _frameBuffer[i];
    }

    return checksum;
}

// Extract checksum value from frame (last 2 bytes, little-endian)
uint16_t FlyskyIBUS::getReceivedCRC()
{
    uint8_t checksumLowByte = _frameBuffer[IBUS_FRAME_LENGTH - 2];
    uint8_t checksumHighByte = _frameBuffer[IBUS_FRAME_LENGTH - 1];

    return (checksumHighByte << 8) | checksumLowByte;
}

// Reset frame buffer and processing state
void FlyskyIBUS::resetIbusBuffer()
{
    _framePosition = 0;
    _frameInProgress = false;

    // Clear buffer
    for (size_t i = 0; i < IBUS_FRAME_LENGTH; i++)
    {
        _frameBuffer[i] = 0;
    }
}

// Initialize all channels to default/neutral values
void FlyskyIBUS::initChannels()
{
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++)
    {
        _channels[i] = IBUS_DEFAULT_VALUE;
        _channelBuffer[i] = IBUS_DEFAULT_VALUE;
    }
}
