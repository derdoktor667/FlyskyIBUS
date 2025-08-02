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
 *        |    |    |<--- Channel Data (28 byte, 2 byte each, 14 chan) --->|  +  <--CRC--->
 *        |    |    |
 *        |    |    Channel 1: Low Byte + High Byte (1000-2000 µs)
 *        |    |    Channel 2: Low Byte + High Byte (1000-2000 µs)
 *        |    |    ...
 *        |    |    Channel 14: Low Byte + High Byte (1000-2000 µs)
 *        |    |
 *        |    Header Byte 2 (0x40)
 *        |
 *        Header Byte 1 (0x20)
 *                                                                        Checksum: 0xFFFF - sum(bytes 0-29)
 *
 * Notes:
 * - Baud rate: 115200, 8N1
 * - Channel values are 16-bit little-endian (low byte first)
 * - Valid channel range: 1000-2000 microseconds
 * - Checksum calculation: 0xFFFF minus sum of all bytes except checksum bytes
 * - Not all 14 channels may be active depending on transmitter configuration
 */

// IBUS protocol constants
#define IBUS_FRAME_LENGTH 32
#define IBUS_HEADER_SIZE 2
#define IBUS_CHECKSUM_SIZE 2
#define IBUS_CHANNEL_DATA_SIZE 28
#define IBUS_MAX_CHANNELS 14
#define IBUS_BYTES_PER_CHANNEL 2

#define IBUS_HEADER_BYTE1 0x20
#define IBUS_HEADER_BYTE2 0x40

#define IBUS_MIN_VALUE 1000
#define IBUS_MAX_VALUE 2000
#define IBUS_DEFAULT_VALUE 1500

#define IBUS_BAUDRATE 115200
#define IBUS_SIGNAL_TIMEOUT 1000

//
class FlyskyIBUS
{
public:
    // Constructor
    FlyskyIBUS();

    // Initialize IBUS with given Serial & Pin
    void begin(HardwareSerial &uart = Serial2, uint8_t rxPin = 16);

    // Handle incoming bytes (ISR Delegate)
    void handleSerialInterrupt();

    // Read channel (1-based index)
    uint16_t readChannel(uint8_t channelNumber);
    void readAllChannels(uint16_t *channelArray);

    // Status queries
    uint8_t getChannelCount();
    bool isConnected();

    // Polling method (optional if no interrupts used)
    void serialEventHandler();

private:
    HardwareSerial *_uart;
    uint8_t _rxPin;

    // Channel data
    uint16_t _channels[IBUS_MAX_CHANNELS];
    uint16_t _channelBuffer[IBUS_MAX_CHANNELS];
    uint8_t _bufferChannelCount;
    uint8_t _channelCount;
    unsigned long _lastFrameTime;

    // Frame processing
    uint8_t _frameBuffer[IBUS_FRAME_LENGTH];
    uint8_t _framePosition;
    bool _frameInProgress;
    bool _headerFound;

    // Internal decoding logic
    void processIbusBytes(uint8_t byte);
    void decodeIbusChannels();
    void copyBufferToChannels();
    void resetIbusBuffer();
    void initChannels();
};
