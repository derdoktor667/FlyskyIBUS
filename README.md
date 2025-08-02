# FlyskyIBUS for ESP32

**Arduino IDE compatible Flysky IBUS receiver library for the ESP32**

This library allows you to receive and decode Flysky IBUS RC signals directly on the ESP32. It is designed to be minimal, fast, and easy to integrate. Fully compatible with the Arduino IDE.

## Features

- 📡 Receive and decode Flysky IBUS frames (up to 14 channels)
- ⚡ Uses direct hardware interrupts for high precision timing
- 🚫 No FreeRTOS dependency
- 🧰 Clean C++ interface, easy to embed and extend
- ✅ Designed for use with the Arduino IDE

### Usage Example

```cpp
#include <Arduino.h>
#include <FlyskyIBUS.h>

FlyskyIBUS ibus;

void setup() {
  Serial.begin(115200);
  ibus.begin(Serial2, 16);  // Set GPIO16 as IBUS RX pin
}

void loop() {
  // Read channel 0 
  uint16_t ch0 = ibus.readChannel(0);
  Serial.println(ch0);

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