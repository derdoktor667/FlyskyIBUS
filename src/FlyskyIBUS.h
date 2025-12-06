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
#include <array>

class FlyskyIBUS
{
public:
    static constexpr auto IBUS_BAUDRATE = 115200;
    static constexpr auto IBUS_MAX_CHANNELS = 0x0E;
    static constexpr auto IBUS_FRAME_START = 0x00;
    static constexpr auto IBUS_FRAME_LENGTH = 0x20;
    static constexpr auto IBUS_HEADER_START = 0x01;
    static constexpr auto IBUS_HEADER_LENGTH = 0x02;
    static constexpr auto IBUS_HEADER_BYTE0 = 0x20;
    static constexpr auto IBUS_HEADER_BYTE1 = 0x40;
    static constexpr auto PAYLOAD_HIGHBYTE = 0x02;
    static constexpr auto PAYLOAD_LOWBYTE = 0x03;
    static constexpr auto IBUS_FAILSAFE_CHANNEL_VALUE = 988;
    static constexpr auto IBUS_FAILSAFE_CHANNEL_COUNT = 4;
    static constexpr auto IBUS_FAILSAVE_SUM = 0x0F70;
    static constexpr auto IBUS_BIT_SHIFT = 0x08;
    static constexpr auto IBUS_CRC_LENGTH = 0x02;
    static constexpr auto IBUS_CRC_MINUEND = 0xFFFF;
    static constexpr auto IBUS_CRC_LOW_BYTE_INDEX = IBUS_FRAME_LENGTH - IBUS_HEADER_LENGTH;
    static constexpr auto IBUS_CRC_HIGH_BYTE_INDEX = IBUS_FRAME_LENGTH - IBUS_HEADER_START;

    // Constructor for the Flysky IBUS Protocol Library
    FlyskyIBUS(HardwareSerial &uart = Serial2, uint8_t rxPin = GPIO_NUM_16, uint8_t txPin = GPIO_NUM_17);

    // Initializes the IBUS receiver
    void begin();

    // Gets the value of a given channel (0-based)
    uint16_t getChannel(const uint8_t channel_nr);

    // Gets all channel values as an array
    std::array<uint16_t, IBUS_MAX_CHANNELS> getChannels();

    // Checks if failsafe is active
    bool isFailsafe() const { return _isFailsafeActive; }

    // Returns the configured RX pin
    uint8_t getRxPin() const { return _rxPin; }

private:
    // --- Hardware Configuration ---
    HardwareSerial *_uart; // Pointer to the HardwareSerial instance used for IBUS communication
    uint8_t _rxPin;        // GPIO pin configured for UART RX
    uint8_t _txPin;        // GPIO pin configured for UART TX

    // --- RX Buffer and Frame Processing ---
    uint8_t _frame_buffer[IBUS_FRAME_LENGTH]; // Buffer to store the incoming IBUS frame bytes
    uint8_t _frame_position;                  // Current position in the frame buffer during reception

    void _processByte(uint8_t byte); // Processes a single incoming byte to build the frame
    void _processFrame();            // Processes a complete received frame
    void _decode_channels();         // Decodes channel values from the processed frame
    void _failsafe_check();          // Checks and updates the failsafe status

    // Decoded channel values (16-bit, 0-based)
    uint16_t _channels[IBUS_MAX_CHANNELS];

    // Failsafe status flag
    bool _isFailsafeActive;
};