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
    static constexpr auto IBUS_FAILSAFE_CHANNEL_VALUE = 988; // Specific value from receiver on failsafe
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
