#include "FlyskyIBUS.h"

// Static instance for interrupt handling
FlyskyIBUS* FlyskyIBUS::_instance = nullptr;

// Constructor - initialize all variables to safe defaults
FlyskyIBUS::FlyskyIBUS() {
    _uart = nullptr;
    _interruptMode = false;
    _newDataAvailable = false;
    _framePosition = 0;
    _frameInProgress = false;
    _validChannelCount = 0;
    _bufferChannelCount = 0;
    _lastValidFrame = 0;
    _lastInterruptFrame = 0;
    _signalPresent = false;
    _debugEnabled = false;
    
    // Set static instance for interrupt handling
    _instance = this;
    
    initializeChannelsToDefault();
    resetFrameBuffer();
}

// Destructor - cleanup interrupts
FlyskyIBUS::~FlyskyIBUS() {
    disableUartInterrupt();
    if (_instance == this) {
        _instance = nullptr;
    }
}

// Initialize with existing UART configuration
void FlyskyIBUS::begin(HardwareSerial& uart) {
    _uart = &uart;
    setupUartInterrupt();
    debugPrint("IBUS initialized with existing UART configuration (interrupt mode)");
}

// Initialize with custom RX pin (TX disabled)
void FlyskyIBUS::begin(HardwareSerial& uart, int8_t rxPin) {
    _uart = &uart;
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, rxPin, -1);
    setupUartInterrupt();
    debugPrint("IBUS initialized with interrupt mode - RX pin: " + String(rxPin));
}

// Initialize with custom RX and TX pins
void FlyskyIBUS::begin(HardwareSerial& uart, int8_t rxPin, int8_t txPin) {
    _uart = &uart;
    _uart->begin(IBUS_BAUDRATE, SERIAL_8N1, rxPin, txPin);
    setupUartInterrupt();
    debugPrint("IBUS initialized with interrupt mode - RX pin: " + String(rxPin) + ", TX pin: " + String(txPin));
}

// Check if new data is available (automatically updated by interrupts)
bool FlyskyIBUS::hasNewData() {
    if (_newDataAvailable) {
        copyChannelsFromBuffer();
        _newDataAvailable = false;
        return true;
    }
    
    // Check for signal timeout
    if (millis() - _lastValidFrame > IBUS_SIGNAL_TIMEOUT) {
        _signalPresent = false;
    }
    
    return false;
}

// Optional manual update function (for compatibility or special cases)
bool FlyskyIBUS::update() {
    if (_interruptMode) {
        // In interrupt mode, just check for new data
        return hasNewData();
    }
    
    // Fallback to polling mode if interrupts not set up
    if (!_uart) {
        return false;
    }
    
    bool newFrameReceived = false;
    
    // Process all available bytes from UART
    while (_uart->available()) {
        uint8_t incomingByte = _uart->read();
        
        if (processIncomingByte(incomingByte)) {
            newFrameReceived = true;
        }
    }
    
    // Check for signal timeout
    if (millis() - _lastValidFrame > IBUS_SIGNAL_TIMEOUT) {
        _signalPresent = false;
    }
    
    return newFrameReceived;
}

// Process a single incoming byte and manage frame assembly (polling mode)
bool FlyskyIBUS::processIncomingByte(uint8_t byte) {
    // Check for frame start sequence
    if (!_frameInProgress) {
        if (_framePosition == 0 && byte == IBUS_HEADER_BYTE1) {
            // First header byte received
            _frameBuffer[_framePosition++] = byte;
            debugPrint("Header byte 1 detected");
        } 
        else if (_framePosition == 1 && byte == IBUS_HEADER_BYTE2) {
            // Second header byte received - frame starts now
            _frameBuffer[_framePosition++] = byte;
            _frameInProgress = true;
            debugPrint("Header complete - frame started");
        } 
        else {
            // Wrong sequence - reset and try again
            resetFrameBuffer();
            if (byte == IBUS_HEADER_BYTE1) {
                _frameBuffer[_framePosition++] = byte;
            }
        }
        return false;
    }
    
    // Frame in progress - collect data
    if (_framePosition < IBUS_FRAME_LENGTH) {
        _frameBuffer[_framePosition++] = byte;
        
        // Check if frame is complete
        if (_framePosition == IBUS_FRAME_LENGTH) {
            bool frameValid = isValidFrame();
            
            if (frameValid) {
                decodeChannels();
                _lastValidFrame = millis();
                _signalPresent = true;
                debugPrint("Valid frame received - " + String(_validChannelCount) + " channels");
            } else {
                debugPrint("Invalid frame - checksum failed");
            }
            
            resetFrameBuffer();
            return frameValid;
        }
    } 
    else {
        // Frame overflow - should not happen with correct implementation
        debugPrint("Frame buffer overflow - resetting");
        resetFrameBuffer();
    }
    
    return false;
}

// Validate frame by checking the checksum
bool FlyskyIBUS::isValidFrame() {
    uint16_t calculatedChecksum = calculateFrameChecksum();
    uint16_t frameChecksum = extractChecksumFromFrame();
    
    if (_debugEnabled) {
        debugPrint("Calculated checksum: 0x" + String(calculatedChecksum, HEX));
        debugPrint("Frame checksum: 0x" + String(frameChecksum, HEX));
    }
    
    return (calculatedChecksum == frameChecksum);
}

// Decode channel data from the validated frame (polling mode)
void FlyskyIBUS::decodeChannels() {
    _validChannelCount = 0;
    
    // Channel data starts after the 2-byte header
    uint8_t channelDataStart = IBUS_HEADER_SIZE;
    
    // Decode each channel (2 bytes per channel, little-endian format)
    for (uint8_t channelIndex = 0; channelIndex < IBUS_MAX_CHANNELS; channelIndex++) {
        uint8_t lowBytePosition = channelDataStart + (channelIndex * IBUS_BYTES_PER_CHANNEL);
        uint8_t highBytePosition = lowBytePosition + 1;
        
        // Check if we have enough data for this channel
        if (highBytePosition >= (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE)) {
            break; // Not enough data for more channels
        }
        
        // Extract channel value (little-endian: low byte first, then high byte)
        uint8_t lowByte = _frameBuffer[lowBytePosition];
        uint8_t highByte = _frameBuffer[highBytePosition];
        uint16_t channelValue = (highByte << 8) | lowByte;
        
        // Validate channel value is within reasonable range
        if (channelValue >= IBUS_MIN_VALUE && channelValue <= IBUS_MAX_VALUE) {
            _channels[channelIndex] = channelValue;
            _validChannelCount++;
            
            if (_debugEnabled) {
                debugPrint("Channel " + String(channelIndex + 1) + ": " + String(channelValue));
            }
        } else {
            // Invalid channel value - stop processing further channels
            debugPrint("Invalid channel value detected: " + String(channelValue));
            break;
        }
    }
}

// Read specific channel value (1-based channel numbering)
uint16_t FlyskyIBUS::readChannel(uint8_t channelNumber) {
    if (channelNumber >= 1 && channelNumber <= IBUS_MAX_CHANNELS) {
        return _channels[channelNumber - 1]; // Convert to 0-based array index
    }
    
    debugPrint("Invalid channel number: " + String(channelNumber));
    return IBUS_DEFAULT_VALUE;
}

// Copy all channel values to user-provided array
void FlyskyIBUS::readAllChannels(uint16_t* channelArray) {
    if (channelArray == nullptr) {
        debugPrint("Null pointer provided to readAllChannels");
        return;
    }
    
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
        channelArray[i] = _channels[i];
    }
}

// Check if IBUS signal is present and recent
bool FlyskyIBUS::isConnected() {
    return _signalPresent && (millis() - _lastValidFrame < IBUS_SIGNAL_TIMEOUT);
}

// Get number of valid channels in last received frame
uint8_t FlyskyIBUS::getChannelCount() {
    return _validChannelCount;
}

// Get time elapsed since last valid frame (in milliseconds)
unsigned long FlyskyIBUS::getLastUpdateTime() {
    return millis() - _lastValidFrame;
}

// Enable or disable debug output
void FlyskyIBUS::enableDebug(bool enable) {
    _debugEnabled = enable;
    debugPrint(_debugEnabled ? "Debug enabled" : "Debug disabled");
}

// Print all channel values to Serial (for debugging)
void FlyskyIBUS::printChannels() {
    if (!Serial) return;
    
    Serial.print("[IBUS] Channels (");
    Serial.print(_validChannelCount);
    Serial.print("): ");
    
    for (uint8_t i = 0; i < _validChannelCount; i++) {
        Serial.print("CH");
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(_channels[i]);
        Serial.print(" ");
    }
    Serial.println();
}

// Setup UART interrupt for automatic data reception
void FlyskyIBUS::setupUartInterrupt() {
    if (!_uart) return;
    
    // Enable UART interrupt for data reception
    #ifdef ESP32
        _uart->onReceive(uartInterruptHandler);
        _interruptMode = true;
        debugPrint("UART interrupt enabled for automatic reception");
    #else
        debugPrint("Interrupt mode not supported on this platform - using polling mode");
        _interruptMode = false;
    #endif
}

// Disable UART interrupt
void FlyskyIBUS::disableUartInterrupt() {
    #ifdef ESP32
        if (_uart && _interruptMode) {
            _uart->onReceive(nullptr);
            _interruptMode = false;
            debugPrint("UART interrupt disabled");
        }
    #endif
}

// Static interrupt handler
void IRAM_ATTR FlyskyIBUS::uartInterruptHandler() {
    if (_instance && _instance->_uart && _instance->_uart->available()) {
        while (_instance->_uart->available()) {
            uint8_t byte = _instance->_uart->read();
            _instance->processIncomingByteISR(byte);
        }
    }
}

// Interrupt-safe byte processing
void FlyskyIBUS::processIncomingByteISR(uint8_t byte) {
    // Check for frame start sequence
    if (!_frameInProgress) {
        if (_framePosition == 0 && byte == IBUS_HEADER_BYTE1) {
            _frameBuffer[_framePosition++] = byte;
        } 
        else if (_framePosition == 1 && byte == IBUS_HEADER_BYTE2) {
            _frameBuffer[_framePosition++] = byte;
            _frameInProgress = true;
        } 
        else {
            _framePosition = 0;
            if (byte == IBUS_HEADER_BYTE1) {
                _frameBuffer[_framePosition++] = byte;
            }
        }
        return;
    }
    
    // Frame in progress - collect data
    if (_framePosition < IBUS_FRAME_LENGTH) {
        _frameBuffer[_framePosition++] = byte;
        
        // Check if frame is complete
        if (_framePosition == IBUS_FRAME_LENGTH) {
            // Validate checksum in interrupt context (fast calculation)
            uint16_t calculatedChecksum = 0xFFFF;
            for (uint8_t i = 0; i < (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE); i++) {
                calculatedChecksum -= _frameBuffer[i];
            }
            
            uint8_t checksumLowByte = _frameBuffer[IBUS_FRAME_LENGTH - 2];
            uint8_t checksumHighByte = _frameBuffer[IBUS_FRAME_LENGTH - 1];
            uint16_t frameChecksum = (checksumHighByte << 8) | checksumLowByte;
            
            if (calculatedChecksum == frameChecksum) {
                decodeChannelsISR();
                _lastInterruptFrame = millis();
                _newDataAvailable = true;
            }
            
            // Reset frame
            _framePosition = 0;
            _frameInProgress = false;
        }
    } 
    else {
        // Frame overflow
        _framePosition = 0;
        _frameInProgress = false;
    }
}

// Interrupt-safe channel decoding
void FlyskyIBUS::decodeChannelsISR() {
    _bufferChannelCount = 0;
    uint8_t channelDataStart = IBUS_HEADER_SIZE;
    
    // Decode channels in interrupt buffer
    for (uint8_t channelIndex = 0; channelIndex < IBUS_MAX_CHANNELS; channelIndex++) {
        uint8_t lowBytePosition = channelDataStart + (channelIndex * IBUS_BYTES_PER_CHANNEL);
        uint8_t highBytePosition = lowBytePosition + 1;
        
        // Check if we have enough data for this channel
        if (highBytePosition >= (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE)) {
            break;
        }
        
        // Extract channel value (little-endian)
        uint8_t lowByte = _frameBuffer[lowBytePosition];
        uint8_t highByte = _frameBuffer[highBytePosition];
        uint16_t channelValue = (highByte << 8) | lowByte;
        
        // Validate channel value
        if (channelValue >= IBUS_MIN_VALUE && channelValue <= IBUS_MAX_VALUE) {
            _channelBuffer[channelIndex] = channelValue;
            _bufferChannelCount++;
        } else {
            // Invalid channel value - stop processing
            break;
        }
    }
}

// Copy channels from interrupt buffer to main buffer (thread-safe)
void FlyskyIBUS::copyChannelsFromBuffer() {
    // Disable interrupts briefly for atomic copy
    noInterrupts();
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
        _channels[i] = _channelBuffer[i];
    }
    _validChannelCount = _bufferChannelCount;
    _lastValidFrame = _lastInterruptFrame;
    _signalPresent = true;
    interrupts();
}

// Calculate checksum for frame validation
uint16_t FlyskyIBUS::calculateFrameChecksum() {
    uint16_t checksum = 0xFFFF;
    
    // Calculate checksum for all bytes except the checksum bytes at the end
    for (uint8_t i = 0; i < (IBUS_FRAME_LENGTH - IBUS_CHECKSUM_SIZE); i++) {
        checksum -= _frameBuffer[i];
    }
    
    return checksum;
}

// Extract checksum value from frame (last 2 bytes, little-endian)
uint16_t FlyskyIBUS::extractChecksumFromFrame() {
    uint8_t checksumLowByte = _frameBuffer[IBUS_FRAME_LENGTH - 2];
    uint8_t checksumHighByte = _frameBuffer[IBUS_FRAME_LENGTH - 1];
    
    return (checksumHighByte << 8) | checksumLowByte;
}

// Reset frame buffer and processing state
void FlyskyIBUS::resetFrameBuffer() {
    _framePosition = 0;
    _frameInProgress = false;
    
    // Clear frame buffer
    memset((void*)_frameBuffer, 0, IBUS_FRAME_LENGTH);
}

// Initialize all channels to default/neutral values
void FlyskyIBUS::initializeChannelsToDefault() {
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
        _channels[i] = IBUS_DEFAULT_VALUE;
        _channelBuffer[i] = IBUS_DEFAULT_VALUE;
    }
}

// Debug print helper function
void FlyskyIBUS::debugPrint(const String& message) {
    if (_debugEnabled && Serial) {
        Serial.print("[IBUS] ");
        Serial.println(message);
    }
}
