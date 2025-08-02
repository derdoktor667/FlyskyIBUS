/**
 * @file read_Channels.ino
 * @brief ESP32 Flysky IBUS Receiver Test – Arduino IDE Compatible
 * @author Wastl Kraus
 * @date 2025-08-01
 * @license MIT
 */

#include <Arduino.h>
#include "FlyskyIBUS.h"

static constexpr HardwareSerial &IBUS_SERIAL = Serial2;
static constexpr uint8_t IBUS_PIN = GPIO_NUM_16;

FlyskyIBUS ibus;

void setup()
{
    Serial.begin(115200);
    delay(500);
    ibus.begin(IBUS_SERIAL, IBUS_PIN);
    Serial.println("Flysky IBUS Plotter");
}

void loop()
{
    static unsigned long lastPlot = 0;

    if (millis() - lastPlot > 50)
    {
        lastPlot = millis();

        if (ibus.isConnected())
        {
            uint16_t channels[IBUS_MAX_CHANNELS];
            ibus.readAllChannels(channels);
            uint8_t count = ibus.getChannelCount();

            for (uint8_t i = 0; i < count; i++)
            {
                Serial.print(channels[i]);
                if (i < count - 1)
                    Serial.print(",");
            }
            Serial.println();
        }
        else
        {
            // Optionale Meldung – nicht plotten!
            Serial.println("0,0,0,0,0,0");
        }
    }
}
