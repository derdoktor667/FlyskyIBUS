/**
 * @file read_Channels.ino
 * @brief ESP32 Library for Flysky IBUS Reception and Decoding – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include <Arduino.h>
#include <FlyskyIBUS.h>

// Default UART2 RX Pin
static constexpr uint8_t IBUS_PIN = 17;

// Use UART2
static constexpr HardwareSerial &IBUS_UART = Serial2;

// Create IBUS Listener
FlyskyIBUS ibus;

//
void setup()
{
    Serial.begin(115200);

    // Init like any serial
    ibus.begin(IBUS_UART, IBUS_PIN);
}

//
void loop()
{
    // Just read channels
    uint16_t channel_01 = ibus.readChannel(1);
    uint16_t channel_02 = ibus.readChannel(2);

    Serial.print("Channel 01: ");
    Serial.println(channel_01);
    Serial.print("Channel 02: ");
    Serial.println(channel_02);
    Serial.println("----------------");

    // You can even use delay
    delay(1000);
}
