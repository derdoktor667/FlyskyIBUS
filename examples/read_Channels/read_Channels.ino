#include <Arduino.h>
#include "FlyskyIBUS.h"

FlyskyIBUS ibus;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== IBUS Interrupt Test ===");
    
    // Initialize IBUS on pin 16 - Timer interrupt handles everything!
    ibus.begin(Serial2, 16);
    Serial.println("IBUS Library initialized with timer interrupt!");
    Serial.println("No update() needed in loop!");
}

void loop() {
    // NO ibus.update() needed! Timer interrupt handles everything.
    
    // Check connection
    if (ibus.isConnected()) {
        // Display all 6 channels every 200ms
        static unsigned long lastDisplay = 0;
        if (millis() - lastDisplay > 200) {
            
            Serial.print("CH1:");
            Serial.print(ibus.readChannel(1));
            Serial.print(" CH2:");
            Serial.print(ibus.readChannel(2));
            Serial.print(" CH3:");
            Serial.print(ibus.readChannel(3));
            Serial.print(" CH4:");
            Serial.print(ibus.readChannel(4));
            Serial.print(" CH5:");
            Serial.print(ibus.readChannel(5));
            Serial.print(" CH6:");
            Serial.print(ibus.readChannel(6));
            Serial.print(" (");
            Serial.print(ibus.getChannelCount());
            Serial.println(" valid)");
            
            lastDisplay = millis();
        }
    } else {
        // Show connection status every 2 seconds
        static unsigned long lastStatus = 0;
        if (millis() - lastStatus > 2000) {
            Serial.println("Waiting for IBUS connection...");
            lastStatus = millis();
        }
    }
    
    // Your main code can run here without worrying about IBUS!
    // The timer interrupt handles all IBUS processing automatically.
    
    delay(10);
}
