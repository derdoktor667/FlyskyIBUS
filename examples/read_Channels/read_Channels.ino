/**
 * @file read_Channels.ino
 * @brief ESP32 Flysky IBUS Receiver Test – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include <Arduino.h>
#include <FlyskyIBUS.h>

// USB serial port settings
static constexpr HardwareSerial &USB_SERIAL = Serial0;
static constexpr uint32_t SERIAL_BAUD = 115200;

// UART Configuration (optional)
static constexpr HardwareSerial &IBUS_SERIAL = Serial2;
static constexpr uint8_t IBUS_PIN = GPIO_NUM_16;

// Creates an IBUS receiver ready to use
// FlyskyIBUS ibus(); // UART2 - Pin 16 default
FlyskyIBUS ibus(IBUS_SERIAL, IBUS_PIN);

//
void setup()
{
    // Starts IBUS receiving
    ibus.begin();

    //
    USB_SERIAL.begin(SERIAL_BAUD);
    USB_SERIAL.println(" === IBUS Receiver initialized === ");
}

//
void loop()
{
    // No further function needed
    // UART interrupt driven receiver

    // Simply get channel values
    uint16_t ch_01 = ibus.getChannel(1);
    uint16_t ch_02 = ibus.getChannel(2);
    uint16_t ch_03 = ibus.getChannel(3);
    uint16_t ch_04 = ibus.getChannel(4);

    // Prints the channels to serial for example
    USB_SERIAL.print(" ");
    USB_SERIAL.print(ch_01);
    USB_SERIAL.print(",");
    USB_SERIAL.print(ch_02);
    USB_SERIAL.print(",");
    USB_SERIAL.print(ch_03);
    USB_SERIAL.print(",");
    USB_SERIAL.print(ch_04);
    USB_SERIAL.println(" ");

    // Not flooding serial port
    delayMicroseconds(1000);
}
