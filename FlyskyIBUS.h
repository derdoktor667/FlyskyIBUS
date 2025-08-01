#ifndef FLYSKY_IBUS_H
#define FLYSKY_IBUS_H

#include <Arduino.h>
#include <HardwareSerial.h>

/*
 * FlyskyIBUS - Arduino library for decoding Flysky IBUS protocol
 * 
 * IBUS Frame Structure (32 bytes total):
 * 
 * Byte:  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
 * Data: 0x20 0x40 [CH1_L][CH1_H][CH2_L][CH2_H][CH3_L][CH3_H][CH4_L][CH4_H][CH5_L][CH5_H][CH6_L][CH6_H][CH7_L][CH7_H][CH8_L][CH8_H][CH9_L][CH9_H][CH10L][CH10H][CH11L][CH11H][CH12L][CH12H][CH13L][CH13H][CH14L][CH14H][CHKL][CHKH]
 *        |    |    |<--- Channel Data (28 bytes, up to 14 channels, 2 bytes each, little-endian) --->|                                                                                                                                              |<-CRC->|
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
#define IBUS_FRAME_LENGTH 32        // Total frame length in bytes
#define IBUS_HEADER_SIZE 2          // Header bytes (0x20, 0x40)
#define IBUS_CHECKSUM_SIZE 2        // Checksum bytes at end
#define IBUS_CHANNEL_DATA_SIZE 28   // Channel data: 14 channels × 2 bytes each
#define IBUS_MAX_CHANNELS 14        // Maximum number of channels supported
#define IBUS_BYTES_PER_CHANNEL 2    // Each channel uses 2 bytes (little-endian)

// IBUS frame header bytes
#define IBUS_HEADER_BYTE1 0x20
#define IBUS_HEADER_BYTE2 0x40

// Channel value ranges (in microseconds)
#define IBUS_MIN_VALUE 1000         // Minimum channel value
#define IBUS_MAX_VALUE 2000         // Maximum channel value
#define IBUS_DEFAULT_VALUE 1500     // Default/neutral channel value

// Communication settings
#define IBUS_BAUDRATE 115200        // Standard IBUS baud rate
#define IBUS_SIGNAL_TIMEOUT 1000    // Signal timeout in milliseconds

class FlyskyIBUS {
public:
    // Constructor
    FlyskyIBUS();
    
    // Destructor - cleanup interrupts
    ~FlyskyIBUS();
    
    // Initialize IBUS with UART (ESP32) - automatic interrupt-based reception
    void begin(HardwareSerial& uart);
    void begin(HardwareSerial& uart, int8_t rxPin);
    void begin(HardwareSerial& uart, int8_t rxPin, int8_t txPin);
    
    // Channel access functions (1-based indexing: channels 1-14)
    uint16_t readChannel(uint8_t channelNumber);
    void readAllChannels(uint16_t* channelArray);
    
    // Status functions
    bool isConnected();
    bool hasNewData();
    uint8_t getChannelCount();
    unsigned long getLastUpdateTime();
    
    // Optional manual update (for compatibility or special cases)
    bool update();
    
    // Debug functions
    void enableDebug(bool enable = true);
    void printChannels();
    
private:
    // UART communication
    HardwareSerial* _uart;
    
    // Interrupt-based reception
    static FlyskyIBUS* _instance;  // For interrupt callback
    bool _interruptMode;
    volatile bool _newDataAvailable;
    
    // Frame processing (volatile for interrupt safety)
    volatile uint8_t _frameBuffer[IBUS_FRAME_LENGTH];
    volatile uint8_t _framePosition;
    volatile bool _frameInProgress;
    
    // Channel data (double buffering for thread safety)
    uint16_t _channels[IBUS_MAX_CHANNELS];
    uint16_t _channelBuffer[IBUS_MAX_CHANNELS];  // Interrupt writes here
    uint8_t _validChannelCount;
    volatile uint8_t _bufferChannelCount;
    
    // Status tracking
    unsigned long _lastValidFrame;
    volatile unsigned long _lastInterruptFrame;
    bool _signalPresent;
    bool _debugEnabled;
    
    // Frame processing methods
    bool processIncomingByte(uint8_t byte);
    void processIncomingByteISR(uint8_t byte);  // Interrupt-safe version
    bool isValidFrame();
    void decodeChannels();
    void decodeChannelsISR();  // Interrupt-safe version
    void resetFrameBuffer();
    void copyChannelsFromBuffer();  // Copy from interrupt buffer to main
    
    // Interrupt handling
    static void IRAM_ATTR uartInterruptHandler();
    void setupUartInterrupt();
    void disableUartInterrupt();
    
    // Utility methods
    uint16_t calculateFrameChecksum();
    uint16_t extractChecksumFromFrame();
    void debugPrint(const String& message);
    void initializeChannelsToDefault();
};

#endif // FLYSKY_IBUS_H
