/**
 * @file FlyskyIBUS.h
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-10-17
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <numeric>
#include <algorithm>

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

//
class FlyskyIBUS
{
public:
    // Flysky IBUS Protocol Library
    FlyskyIBUS(HardwareSerial &uart = Serial2, uint8_t rxPin = GPIO_NUM_16, uint8_t txPin = GPIO_NUM_17);

    // Starts the IBUS receiver
    void begin();

    // Get value of given channel (0-based)
    uint16_t getChannel(const uint8_t channel_nr);

    // Check status
    bool isFailsafe() const { return _isFailsafeActive; }

    // Returns the configured RX pin
    uint8_t getRxPin() const { return _rxPin; }

private:
    // --- IBUS protocol ---
    static constexpr auto IBUS_BAUDRATE = 115200;
    static constexpr auto IBUS_FRAME_LENGTH = 32;
    static constexpr auto IBUS_MAX_CHANNELS = 14;
    static constexpr auto IBUS_HEADER_BYTE0 = 0x20;
    static constexpr auto IBUS_HEADER_BYTE1 = 0x40;
    static constexpr auto IBUS_HEADER_START = 1;
    static constexpr auto IBUS_HEADER_LENGTH = 2;
    static constexpr auto IBUS_FRAME_START = 0;
    static constexpr auto IBUS_CRC_MINUEND = 0xFFFF;
    static constexpr auto IBUS_CRC_SHIFT = 0x08;
    static constexpr auto IBUS_CRC_LENGTH = 2;
    static constexpr auto PAYLOAD_HIGHBYTE = 2;
    static constexpr auto PAYLOAD_LOWBYTE = 3;
    static constexpr auto IBUS_FAILSAVE_SUM = 0xF70;
    static constexpr auto IBUS_FAILSAFE_CHANNEL_COUNT = 4;
    static constexpr auto IBUS_FAILSAFE_CHANNEL_VALUE = 988;
    static constexpr auto IBUS_CHECKSUM_LOW_BYTE_INDEX = IBUS_FRAME_LENGTH - 2;
    static constexpr auto IBUS_CHECKSUM_HIGH_BYTE_INDEX = IBUS_FRAME_LENGTH - 1;
    static constexpr auto IBUS_BYTE_SHIFT = IBUS_CRC_SHIFT;

    // --- Hardware Config ---
    HardwareSerial *_uart;
    uint8_t _rxPin;
    uint8_t _txPin;

    // --- RX Buffer ---
    uint8_t _frame_buffer[IBUS_FRAME_LENGTH];
    uint8_t _frame_position;

    void _generateFrame(uint8_t byte);
    void _decode_channels();
    void _failsafe_check();

    // Decoded channel values
    uint16_t _channels[IBUS_MAX_CHANNELS];

    // Failsafe
    bool _isFailsafeActive;
};
