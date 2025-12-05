/**
 * @file FlyskyIBUS.h
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-10-17
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "src/FlyskyIBUS.h"
 
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
