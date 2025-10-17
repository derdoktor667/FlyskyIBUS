# FlyskyIBUS Library for ESP32

**Arduino IDE compatible Flysky IBUS receiver library for ESP32**

Receive and decode Flysky IBUS RC signals directly on the ESP32. Interrupt-driven, non-blocking, and easy to use.

## Features

- 📡 **Full IBUS Support** – Receive and decode all 14 channels (1000-2000µs)
- ⚡ **Hardware UART Interrupts** – Precise timing via direct ESP32 UART interrupts
- 🚫 **Non-blocking Design** – No timers, no delays, runs completely in the background
- 🛡️ **Robust Failsafe** – Dual failsafe detection for maximum safety
- 🛠️ **Plug & Play** – One function call is enough, the library handles the rest
- 🔧 **Configurable** – Choose any UART interface and GPIO pin
- ✅ **Arduino IDE Ready** – Fully compatible with Arduino IDE and ESP32 Core

## Failsafe

The library implements a dual-logic failsafe for maximum reliability:
1.  **Signal Loss (Timeout):** If no valid IBUS data is received for more than 100ms, the failsafe is triggered.
2.  **Receiver Failsafe Signal:** The library detects when the receiver itself enters failsafe mode and sends channel values outside the standard range (1000-2000µs).

When failsafe is active, `getChannel()` will return the neutral value (1500), and `hasFailsafe()` will return `true`.

## Installation

Copy the library to your Arduino `libraries` folder and restart the Arduino IDE.

## Usage

```cpp
#include <FlyskyIBUS.h>

// Default: Serial2, GPIO16
FlyskyIBUS ibus;

void setup() {
  Serial.begin(115200);
  ibus.begin();
}

void loop() {
  uint16_t ch1 = ibus.getChannel(1);
  uint16_t ch2 = ibus.getChannel(2);
  
  Serial.print(" CH1: "); Serial.println(ch1);
  Serial.print(" CH2: "); Serial.println(ch2);
  
  delay(100);
}
```

## IBUS Frame Structure

```
Byte:  0    1     2      3      4      5    ...    28     29     30    31
Data: 0x20 0x40 [CH1_L][CH1_H][CH2_L][CH2_H] ... [CH14L][CH14H][CHKL][CHKH]
       |    |    |<--- Channel Data (28 Bytes, 14 Channels) --->|  |<--CRC-->|
       |    |
       Header (0x20 0x40)
```

**115200 Baud, 8N1 • Channel values: Little-Endian • CRC: 0xFFFF - sum of all bytes**

## Hardware Connection

```
Flysky Receiver    ESP32
[IBUS] ---------> [GPIO16]
[GND]  ---------> [GND]
[VCC]  ---------> [3.3V/5V]
```

## API

```cpp
FlyskyIBUS ibus(Serial2, GPIO_NUM_16);  // Constructor
bool begin();                           // Initialization
uint16_t getChannel(uint8_t channel);   // Read channel (1-14), returns 1500 on failsafe
uint16_t getRawChannel(uint8_t ch);     // Read channel (1-14), returns raw value
uint8_t getChannelCount();              // Number of active channels
bool hasFailsafe();                     // Returns true if failsafe is active
```

## 📄 License

MIT License – see [LICENSE](LICENSE)

---

## 👤 Author

**Wastl Kraus**  
GitHub: [@derdoktor667](https://github.com/derdoktor667)  
Website: [wir-sind-die-matrix.de](https://wir-sind-die-matrix.de)