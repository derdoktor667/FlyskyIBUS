# FlyskyIBUS Library for ESP32

**Arduino IDE compatible Flysky IBUS receiver library for the ESP32**

This library allows you to receive and decode Flysky IBUS RC signals directly on the ESP32. It is designed to be minimal, fast, and easy to integrate. Fully compatible with the Arduino IDE.

## Features

- 📡 Receive and decode Flysky IBUS Signal (up to 14 channels)
- ⚡ Uses direct hardware interrupts for high precision timing
- 🚫 Non - blocking and no timer usage
- 🛠️ Easy usage, just setup and fotget
- 🧰 Clean C++ interface, easy to embed and extend
- ✅ Designed for use with the Arduino IDE

### Usage Example

```cpp
#include <Arduino.h>
#include <FlyskyIBUS.h>

// Using UART2 / GPIO16 
// FlyskyIBUS(Serial2, GPIO_NUM_16)
FlyskyIBUS ibus();

//
void setup() {
  Serial.begin(115200);

  // Start and forget
  ibus.begin();
}

//
void loop() {
  // No loop - functions needed, just read the channel values

  // Read channel number and print it
  uint16_t ch_01 = ibus.readChannel(1);
  Serial.println(ch_01);

  // You can use delay as well
  delay(1000);
}
```
---

## 📄 License

MIT License – see [LICENSE](LICENSE)

---

## 👤 Author

**Wastl Kraus**  
GitHub: [@derdoktor667](https://github.com/derdoktor667)  
Website: [wir-sind-die-matrix.de](https://wir-sind-die-matrix.de)