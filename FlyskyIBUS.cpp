/**
 * @file FlyskyIBUS.cpp - Native UART Interrupt Version
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include <Arduino.h>
#include <FlyskyIBUS.h>
#include <HardwareSerial.h>
#include "driver/uart.h"
#include "soc/uart_reg.h"

// Static instance for interrupt handling
FlyskyIBUS *FlyskyIBUS::_instance = nullptr;
uart_port_t FlyskyIBUS::_uartPort = UART_NUM_MAX;

// Constructor
FlyskyIBUS::FlyskyIBUS()
{
    _uart = nullptr;
    _framePosition = 0;
    _frameInProgress = false;
    _bufferChannelCount = 0;
    _channelCount = 0;
    _lastFrameTime = 0;
    _headerFound = false;
    _rxPin = -1;

    // Set static instance
    _instance = this;

    // Prepare Containers
    initChannels();
    resetIbusBuffer();
}

// Initialize with custom RX pin (TX disabled)
void FlyskyIBUS::begin(HardwareSerial &uart, uint8_t rxPin)
{
    _uart = &uart;
    _rxPin = rxPin;
    
    // Determine UART port number
    if (&uart == &Serial) {
        _uartPort = UART_NUM_0;
    } else if (&uart == &Serial1) {
        _uartPort = UART_NUM_1;
    } else if (&uart == &Serial2) {
        _uartPort = UART_NUM_2;
    } else {
        Serial.println("ERROR: Invalid UART port!");
        return;
    }
    
    // Configure UART with ESP-IDF for interrupt support
    setupUartInterrupt();
    
    Serial.println("IBUS: Initialized with native UART interrupt");
}

// Setup native UART interrupt
void FlyskyIBUS::setupUartInterrupt()
{
    // UART configuration
    uart_config_t uart_config = {
        .baud_rate = IBUS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(_uartPort, &uart_config));
    
    // Set UART pins (TX disabled with -1)
    ESP_ERROR_CHECK(uart_set_pin(_uartPort, -1, _rxPin, -1, -1));
    
    // Install UART driver with interrupt
    ESP_ERROR_CHECK(uart_driver_install(_uartPort, 
        IBUS_FRAME_LENGTH * 2,  // RX buffer size
        0,                      // TX buffer size (not used)
        0,                      // Event queue size (not used)
        NULL,                   // Event queue handle
        0                       // Interrupt flags
    ));
    
    // Register interrupt handler
    ESP_ERROR_CHECK(uart_isr_free(_uartPort));
    ESP_ERROR_CHECK(uart_isr_register(_uartPort, 
        uartInterruptHandler,   // Interrupt handler
        NULL,                   // Handler argument
        ESP_INTR_FLAG_IRAM,     // Run in IRAM
        NULL                    // Handle (not needed)
    ));
    
    // Enable RX interrupt
    uart_enable_intr_mask(_uartPort, UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M);
}

// Native UART interrupt handler
void IRAM_ATTR FlyskyIBUS::uartInterruptHandler(void *arg)
{
    if (!_instance) return;
    
    // Get interrupt status
    uint32_t uart_intr_status = uart_ll_get_intsts_mask(&UART0 + _uartPort);
    
    // Clear interrupts
    uart_ll_clr_intsts_mask(&UART0 + _uartPort, uart_intr_status);
    
    // Check if RX interrupt
    if (uart_intr_status & (UART_RXFIFO_FULL_INT_ST_M | UART_RXFIFO_TOUT_INT_ST_M)) {
        // Read all available data
        uint8_t data[32];
        int length = uart_read_bytes(_uartPort, data, sizeof(data), 0);
        
        // Process each byte
        for (int i = 0; i < length; i++) {
            _instance->processIbusBytes(data[i]);
        }
    }
}

// Read specific channel value (1-based channel numbering)
uint16_t FlyskyIBUS::readChannel(uint8_t channelNumber)
{
    if (channelNumber >= 1 && channelNumber <= IBUS_MAX_CHANNELS)
    {
        // Disable interrupts for atomic read
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);
        uint16_t value = _channels[channelNumber - 1];
        portEXIT_CRITICAL(&mux);
        return value;
    }
    return IBUS_DEFAULT_VALUE;
}

// Copy all channel values to user-provided array
void FlyskyIBUS::readAllChannels(uint16_t *channelArray)
{
    if (channelArray == nullptr)
        return;

    // Disable interrupts for atomic read
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++)
    {
        channelArray[i] = _channels[i];
    }
    
    portEXIT_CRITICAL(&mux);
}

// Get number of valid channels received
uint8_t FlyskyIBUS::getChannelCount()
{
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    uint8_t count = _channelCount;
    portEXIT_CRITICAL(&mux);
    return count;
}

// Check if we have received valid data recently
bool FlyskyIBUS::isConnected()
{
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    unsigned long lastFrame = _lastFrameTime;
    portEXIT_CRITICAL(&mux);
    
    return (millis() - lastFrame) < IBUS_SIGNAL_TIMEOUT;
}

// Process incoming bytes (called from interrupt)
void IRAM_ATTR FlyskyIBUS::processIbusBytes(uint8_t byte)
{
    // Store byte in buffer
    _frameBuffer[_framePosition] = byte;
    _framePosition++;

    // Look for header pattern at any position
    if (_framePosition >= 2)
    {
        if (_frameBuffer[_framePosition-2] == IBUS_HEADER_BYTE1 && 
            _frameBuffer[_framePosition-1] == IBUS_HEADER_BYTE2)
        {
            // Found header - reset to start of frame
            _frameBuffer[0] = IBUS_HEADER_BYTE1;
            _frameBuffer[1] = IBUS_HEADER_BYTE2;
            _framePosition = 2;
            _frameInProgress = true;
            _headerFound = true;
        }
    }

    // If we have a complete frame
    if (_frameInProgress && _framePosition >= IBUS_FRAME_LENGTH)
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
            // Valid frame - decode channels
            decodeIbusChannels();
            copyBufferToChannels();
            _lastFrameTime = millis();
        }

        // Reset for next frame
        _framePosition = 0;
        _frameInProgress = false;
        _headerFound = false;
    }

    // Prevent buffer overflow
    if (_framePosition >= IBUS_FRAME_LENGTH)
    {
        _framePosition = 0;
        _frameInProgress = false;
        _headerFound = false;
    }
}

// Decode channel data from frame buffer (called from interrupt)
void IRAM_ATTR FlyskyIBUS::decodeIbusChannels()
{
    _bufferChannelCount = 0;

    // Decode all 14 channels
    for (uint8_t channelIndex = 0; channelIndex < IBUS_MAX_CHANNELS; channelIndex++)
    {
        uint8_t lowBytePosition = IBUS_HEADER_SIZE + (channelIndex * IBUS_BYTES_PER_CHANNEL);
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

        // Store channel value
        _channelBuffer[channelIndex] = channelValue;
        
        // Count valid channels
        if (channelValue >= IBUS_MIN_VALUE && channelValue <= IBUS_MAX_VALUE)
        {
            _bufferChannelCount++;
        }
    }
}

// Copy buffer data to main channels (called from interrupt)
void IRAM_ATTR FlyskyIBUS::copyBufferToChannels()
{
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++)
    {
        _channels[i] = _channelBuffer[i];
    }
    _channelCount = _bufferChannelCount;
}

// Reset frame buffer and processing state
void FlyskyIBUS::resetIbusBuffer()
{
    _framePosition = 0;
    _frameInProgress = false;
    _headerFound = false;

    for (size_t i = 0; i < IBUS_FRAME_LENGTH; i++)
    {
        _frameBuffer[i] = 0;
    }
}

// Initialize all channels to default values
void FlyskyIBUS::initChannels()
{
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++)
    {
        _channels[i] = IBUS_DEFAULT_VALUE;
        _channelBuffer[i] = IBUS_DEFAULT_VALUE;
    }
}

// Utility functions
uint16_t FlyskyIBUS::calculateCRC()
{
    uint16_t checksum = 0xFFFF;
    for (uint8_t i = 0; i < (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE); i++)
    {
        checksum -= _frameBuffer[i];
    }
    return checksum;
}

uint16_t FlyskyIBUS::getReceivedCRC()
{
    uint8_t checksumLowByte = _frameBuffer[IBUS_FRAME_LENGTH - 2];
    uint8_t checksumHighByte = _frameBuffer[IBUS_FRAME_LENGTH - 1];
    return (checksumHighByte << 8) | checksumLowByte;
}