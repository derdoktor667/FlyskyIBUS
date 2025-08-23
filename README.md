# FlyskyIBUS Library for ESP32

**Arduino IDE compatible Flysky IBUS receiver library for ESP32**

Receive and decode Flysky IBUS RC signals directly on the ESP32. Interrupt-driven, non-blocking, and easy to use.

## Features

- 📡 **Full IBUS Support** – Receive and decode all 14 channels (1000-2000µs)
- ⚡ **Hardware UART Interrupts** – Precise timing via direct ESP32 UART interrupts
- 🚫 **Non-blocking Design** – No timers, no delays, runs completely in the background
- 🛠️ **Plug & Play** – One function call is enough, the library handles the rest
- 🔧 **Configurable** – Choose any UART interface and GPIO pin
- ✅ **Arduino IDE Ready** – Fully compatible with Arduino IDE and ESP32 Core

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
uint16_t getChannel(uint8_t channel);   // Read channel (1-14)
uint8_t getChannelCount();              // Number of active channels
```

## 📄 License

MIT License – see [LICENSE](LICENSE)

---

## 👤 Author

**Wastl Kraus**  
GitHub: [@derdoktor667](https://github.com/derdoktor667)  
Website: [wir-sind-die-matrix.de](https://wir-sind-die-matrix.de)