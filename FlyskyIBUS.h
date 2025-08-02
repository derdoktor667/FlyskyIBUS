/**
 * @file FlyskyIBUS.h
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

/*
 * FlyskyIBUS - Arduino library for decoding Flysky IBUS protocol
 *
 * IBUS Frame Structure (32 bytes total):
 *
 * Byte:  0    1     2      3      4      5    ...    26     27     28     29     30    31
 * Data: 0x20 0x40 [CH1_L][CH1_H][CH2_L][CH2_H] ... [CH13L][CH13H][CH14L][CH14H][CHKL][CHKH]
 *        |    |    |<--- Channel Data (28 byte, 2 byte each, 14 chan) --->|  +  <--CRC--->                                                                                                                                     |<-CRC->|
 *        |    |    |                                                                                                                                                                                                                              |
 *        |    |    Channel 1: Low Byte + High Byte (1000-2000 µs)                                                                                                                                                                              |
 *        |    |    Channel 2: Low Byte + High Byte (1000-2000 µs)                                                                                                                                                                              |
 *        |    |    ...                                                                                                                                                                                                                          |
 *        |    |    Channel 14: Low Byte + High Byte (1000-2000 µs)                                                                                                                                                                             |
 *        |    |                                                                                                                                                                                                                                 |
 *        |    Header Byte 2 (0x40)                                                                                                                                                                                                             |
 *        |                                                                                                                                                                                                                                      |
 *        Header Byte 1 (0x20)                                                                                                                                                                                                                  |
 *                                                                                                                                                                                                                                               |
 *                                                                                                                                                                                                                           Checksum: 0xFFFF - sum(bytes 0-29)
 *
 * Notes:
 * - Baud rate: 115200, 8N1
 * - Channel values are 16-bit little-endian (low byte first)
 * - Valid channel range: 1000-2000 microseconds
 * - Checksum calculation: 0xFFFF minus sum of all bytes except checksum bytes
 * - Not all 14 channels may be active depending on transmitter configuration
 */

// IBUS protocol constants
#define IBUS_FRAME_LENGTH 32      // Total frame length in bytes
#define IBUS_HEADER_SIZE 2        // Header bytes (0x20, 0x40)
#define IBUS_CHECKSUM_SIZE 2      // Checksum bytes at end
#define IBUS_CHANNEL_DATA_SIZE 28 // Channel data: 14 channels × 2 bytes each
#define IBUS_MAX_CHANNELS 14      // Maximum number of channels supported
#define IBUS_BYTES_PER_CHANNEL 2  // Each channel uses 2 bytes (little-endian)

// IBUS frame header bytes
#define IBUS_HEADER_BYTE1 0x20
#define IBUS_HEADER_BYTE2 0x40

// Channel value ranges (in microseconds)
#define IBUS_MIN_VALUE 1000     // Minimum channel value
#define IBUS_MAX_VALUE 2000     // Maximum channel value
#define IBUS_DEFAULT_VALUE 1500 // Default/neutral channel value

// Communication settings
#define IBUS_BAUDRATE 115200     // Standard IBUS baud rate
#define IBUS_SIGNAL_TIMEOUT 1000 // Signal timeout in milliseconds

class FlyskyIBUS
{
public:
    // Constructor
    FlyskyIBUS();

    // Initialize IBUS with UART
    void begin(HardwareSerial &uart, uint8_t rxPin);

    // Channel access functions (1-based indexing: channels 1-14)
    uint16_t readChannel(uint8_t channelNumber);
    void readAllChannels(uint16_t *channelArray);

    // Status functions
    uint8_t getChannelCount();

private:
    // UART communication
    HardwareSerial *_uart;

    // Interrupt callback
    static FlyskyIBUS *_instance;

    // Frame processing (volatile for interrupt safety)
    volatile uint8_t _frameBuffer[IBUS_FRAME_LENGTH];
    volatile uint8_t _framePosition;
    volatile bool _frameInProgress;

    // Channel data (buffered)
    uint16_t _channels[IBUS_MAX_CHANNELS];
    uint16_t _channelBuffer[IBUS_MAX_CHANNELS]; // Interrupt writes here
    volatile uint8_t _bufferChannelCount;

    // IBUS handling
    void processIbusBytes(uint8_t byte);
    void decodeIbusChannels();
    void resetIbusBuffer();

    // Interrupt handling
    static void IRAM_ATTR ibusHandler();
    void installIbusInterrupt();

    // Utility methods
    uint16_t calculateCRC();
    uint16_t getReceivedCRC();
    void initChannels();
};