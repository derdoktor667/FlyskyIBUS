# FlyskyIBUS Library für ESP32

**Arduino IDE kompatible Flysky IBUS Empfänger-Bibliothek für den ESP32**

Empfang und Dekodierung von Flysky IBUS RC-Signalen direkt auf dem ESP32. Interrupt-gesteuert, non-blocking und einfach zu verwenden.

## Features

- 📡 **Vollständiger IBUS Support** - Empfang und Dekodierung aller 14 Kanäle (1000-2000µs)
- ⚡ **Hardware UART Interrupts** - Präzise Timing durch direkte ESP32 UART Interrupts
- 🚫 **Non-blocking Design** - Keine Timer, keine Delays, arbeitet komplett im Hintergrund
- 🛠️ **Plug & Play** - Ein Funktionsaufruf genügt, Library übernimmt den Rest
- 🔧 **Konfigurierbar** - Beliebige UART-Schnittstelle und GPIO-Pin wählbar
- ✅ **Arduino IDE Ready** - Vollständig kompatibel mit Arduino IDE und ESP32 Core

## Installation

Bibliothek in Arduino `libraries` Ordner kopieren und Arduino IDE neustarten.

## Verwendung

```cpp
#include <FlyskyIBUS.h>

// Standard: Serial2, GPIO16
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

## IBUS Frame-Struktur

```
Byte:  0    1     2      3      4      5    ...    28     29     30    31
Data: 0x20 0x40 [CH1_L][CH1_H][CH2_L][CH2_H] ... [CH14L][CH14H][CHKL][CHKH]
       |    |    |<--- Kanaldaten (28 Bytes, 14 Kanäle) --->|  |<--CRC-->|
       |    |
       Header (0x20 0x40)
```

**115200 Baud, 8N1 • Kanalwerte: Little-Endian • CRC: 0xFFFF - Summe aller Bytes**

## Hardware-Verbindung

```
Flysky Receiver    ESP32
[IBUS] ---------> [GPIO16]
[GND]  ---------> [GND]
[VCC]  ---------> [3.3V/5V]
```

## API

```cpp
FlyskyIBUS ibus(Serial2, GPIO_NUM_16);  // Konstruktor
bool begin();                           // Initialisierung
uint16_t getChannel(uint8_t channel);   // Kanal lesen (1-14)
uint8_t getChannelCount();              // Anzahl aktiver Kanäle
```

## 📄 License

MIT License – see [LICENSE](LICENSE)

---

## 👤 Author

**Wastl Kraus**  
GitHub: [@derdoktor667](https://github.com/derdoktor667)  
Website: [wir-sind-die-matrix.de](https://wir-sind-die-matrix.de)
