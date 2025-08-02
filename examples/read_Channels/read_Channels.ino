/**
 * @file read_Channels.ino
 * @brief ESP32 Flysky IBUS Receiver Test – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include <Arduino.h>
#include <FlyskyIBUS.h>

static constexpr HardwareSerial &USB_SERIAL = Serial0;
static constexpr HardwareSerial &IBUS_SERIAL = Serial2;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint8_t IBUS_PIN = GPIO_NUM_16;

//
FlyskyIBUS ibus;

//
void setup()
{

    //
    USB_SERIAL.begin(SERIAL_BAUD);

    //
    ibus.begin(IBUS_SERIAL, IBUS_PIN);

    //
    USB_SERIAL.println("IBUS Interrupt Receiver Ready");
}

//
void loop()
{
    // Read Channels
    printChannels();
}

//
void printChannels()
{
    static unsigned long lastTime = 0;

    if (millis() - lastTime > 10)
    {
        lastTime = millis();

        if (ibus.isConnected())
        {
            for (size_t i = 0; i < ibus.getChannelCount(); i++)
            {
                USB_SERIAL.print(ibus.readChannel(i + 1));

                if (i < ibus.getChannelCount() - 1)
                    USB_SERIAL.print(",");
            }

            USB_SERIAL.print(" - ");
            USB_SERIAL.print(ibus.getChannelCount());
            USB_SERIAL.println();
        }
        else
        {
            USB_SERIAL.println("IBUS Signal Lost");
        }
    }
}