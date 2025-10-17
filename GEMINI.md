# Gemini Project Context: FlyskyIBUS

## Project Overview

This project is a C++ library for the Arduino framework, specifically designed for ESP32 microcontrollers. Its purpose is to receive and decode the Flysky IBUS remote control protocol. The library utilizes hardware UART interrupts for a non-blocking, efficient, and precise operation, making it suitable for real-time applications like controlling drones or other RC vehicles. It can decode up to 14 channels and is configurable to use any available UART interface and GPIO pin on the ESP32. A key feature is its robust, dual-logic failsafe system that detects both signal loss via timeout and explicit failsafe signals sent by the receiver.

**Key Technologies:**
*   **Language:** C++
*   **Framework:** Arduino (for ESP32)
*   **Core Hardware:** ESP32

**Architecture:**
*   The library is encapsulated in a `FlyskyIBUS` class.
*   The `begin()` method initializes a hardware serial port and attaches an interrupt service routine (`_ibus_handle`).
*   The interrupt handler reads incoming bytes from the UART, assembles them into a 32-byte IBUS frame, validates the checksum, and decodes the channel data.
*   The main application can then poll the `getChannel(ch)` method to retrieve the latest value for a specific channel without being blocked by the reception logic.
*   Failsafe is handled by checking for both a timeout (>100ms since last valid frame) and for channel values sent outside the normal 1000-2000μs range.

## Building and Running

The project uses the `arduino-cli` for building and quality control, as defined in the `.github/workflows/ci.yml` file.

*   **Building the Example:** To compile the provided example sketch, run the following command. This is the primary way to verify the library's integrity.
    ```bash
    arduino-cli compile --fqbn esp32:esp32:esp32 examples/read_Channels/read_Channels.ino
    ```

*   **Running Tests (Quality Checks):** The project has a CI pipeline that performs linting and static analysis.
    *   **Linting:**
        ```bash
        # Requires arduino-lint-action setup, typically run via GitHub Actions
        arduino-lint --compliance strict
        ```
    *   **Static Analysis:**
        ```bash
        # Requires cppcheck to be installed
        cppcheck --enable=warning,performance --std=c++17 --language=c++ ./FlyskyIBUS.cpp ./FlyskyIBUS.h
        ```

## Development Conventions

*   **Code Style:** The code is written in an object-oriented style and is well-documented using Doxygen-style block comments in the header (`.h`) and source (`.cpp`) files.
*   **Dependency Management:** The project is a self-contained Arduino library. The `library.properties` file defines its metadata for the Arduino IDE and Library Manager.
*   **Testing:** Verification is done by compiling the example sketch (`read_Channels.ino`). The CI workflow automates this process to ensure that changes do not break the build.
*   **API Design:** The public API is simple and focused:
    *   `FlyskyIBUS(uart, pin)`: Constructor to specify UART and pin.
    *   `begin()`: Initializes the library.
    *   `getChannel(channel)`: Retrieves the value for a given channel. Returns a safe default value if failsafe is active.
    *   `getRawChannel(channel)`: Retrieves the raw, unfiltered value for a given channel, bypassing the failsafe check.
    *   `hasFailsafe()`: Returns true if a failsafe condition (either timeout or value-based) is detected.
*   **Interrupts:** The library is designed to be interrupt-driven (`IRAM_ATTR`) to avoid blocking the main `loop()` and ensure responsive RC signal processing.
