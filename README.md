# FlyskyIBUS Library for ESP32

[![Arduino Library Badge](https://www.ardu-badge.com/badge/FlyskyIBUS.svg)](https://www.ardu-badge.com/FlyskyIBUS)

The FlyskyIBUS library is a powerful and user-friendly Arduino library for the ESP32, specifically designed for receiving and decoding Flysky iBUS RC signals. It enables seamless integration of your Flysky remote control into your ESP32-based projects, ideal for drones, robots, and other remote-controlled applications.

## ✨ Features at a Glance

*   **Full iBUS Protocol Support**: Receives and processes up to 14 channels from your Flysky iBUS remote control.
*   **Fire and Forget Operation**: No need to call any explicit `update()` or `loop()` function. Simply retrieve channel values using `getChannel()` or `getChannels()`, and the library will automatically process all available incoming iBUS data, ensuring your main loop remains clean and responsive.
*   **Precise Timing**: Leverages hardware UART for reliable and accurate iBUS signal decoding. Data acquisition is seamlessly integrated into channel retrieval, ensuring up-to-date values without explicit polling.
*   **Robust Failsafe Detection**: Integrated dual-logic failsafe detection provides additional safety in critical situations.
*   **Flexible Configuration**: Easy adaptation of the UART interface and GPIO pins to your specific hardware.
*   **Arduino IDE Compatibility**: Seamless integration into your existing Arduino projects and workflows.

## 🤝 Compatibility

This library is specifically developed for **ESP32 microcontrollers** and uses the **Arduino ESP32 Framework**.

## 🚀 Installation

Installation is quick and easy via the Arduino Library Manager:

1.  Open the **Arduino IDE**.
2.  Navigate to `Sketch` -> `Include Library` -> `Manage Libraries...`.
3.  Search for "**FlyskyIBUS**" in the Library Manager.
4.  Select the library from the list and click **"Install"**.

Alternatively, you can manually install the library by cloning or downloading the repository and moving its contents into your Arduino libraries folder (e.g., `~/Arduino/libraries/FlyskyIBUS/` on Linux/macOS or `Documents\Arduino\libraries\` on Windows).

## 💡 Usage Example

Here's a simple example demonstrating how to read channel values with the library:

```cpp
#include <FlyskyIBUS.h>

// Create an IBUS object for Serial2
// Default pins for Serial2 are GPIO16 (RX) and GPIO17 (TX) on many ESP32 boards.
// Adjust the pins to your board if necessary. The TX pin is optional
// if you only want to receive.
FlyskyIBUS IBUS;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting FlyskyIBUS Receiver...");

  // Initialize IBUS on HardwareSerial Serial2, specifying RX and TX pins.
  // The TX pin (GPIO17 in this example) is optional and only required if you intend to transmit data.
  IBUS.begin(Serial2, 16, 17); // Example: Serial2, RX: GPIO16, TX: GPIO17
}

void loop() {
  // Retrieve channel values. The library automatically processes any incoming iBUS data
  // when getChannel() or getChannels() is called, so no explicit update function is needed.
  Serial.print("Channel 1 (Roll): ");
  Serial.println(IBUS.getChannel(0));

  Serial.print("Channel 2 (Pitch): ");
  Serial.println(IBUS.getChannel(1));

  Serial.print("Channel 3 (Throttle): ");
  Serial.println(IBUS.getChannel(2));

  Serial.print("Channel 4 (Yaw): ");
  Serial.println(IBUS.getChannel(3));

  // Optional: Read more channels
  // ...

  // Check the failsafe status
  if (IBUS.isFailsafe()) {
    Serial.println("!!! FAILSAFE ACTIVE !!! Signal loss or error detected.");
  }

  delay(50); // A small delay to avoid flooding the serial output
}
```

You can find more detailed examples in the `examples` folder of the library.

## 💖 Contributing

Contributions, whether in the form of bug reports, feature suggestions, or code improvements, are highly welcome! Please open an Issue or create a Pull Request on GitHub.

## 📄 License

This project is licensed under the MIT License. Details can be found in the [LICENSE](LICENSE) file.

---

## 👩‍💻 For Developers

Here are the `arduino-cli` commands for compiling and uploading example sketches. These are useful for development and automation.

**Compiling an example sketch:**
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 ~/Arduino/libraries/FlyskyIBUS/examples/read_Channels/read_Channels.ino
```

**Uploading to ESP32 (assuming `/dev/ttyUSB0` is the upload port):**
```bash
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 ~/Arduino/libraries/FlyskyIBUS/examples/read_Channels/read_Channels.ino
```