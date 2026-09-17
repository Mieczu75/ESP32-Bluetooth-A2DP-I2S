# ESP32 Bluetooth A2DP I2S

Nadajnik Bluetooth A2DP oparty na ESP32-WROOM-32U. Program odbiera dźwięk PCM z urządzenia Maestro przez I2S i przesyła go do głośnika Bluetooth.

##Najważniejsze funkcje:

- ESP32-WROOM-32U jako nadajnik Bluetooth Classic A2DP,
- wejście audio I2S stereo 16-bit,
- automatyczne wykrywanie 44,1/48 kHz,
- konwersja 48 kHz → 44,1 kHz,
- pamięć 3 ostatnich odbiorników Bluetooth,
- priorytet ostatnio używanego urządzenia,
- automatyczny reconnect po utracie połączenia,
- automatyczne wyszukiwanie nowego odbiornika,
- filtrowanie urządzeń Bluetooth Audio/Video,
- pełna regeneracja stosu Bluetooth po utracie aktywnego połączenia,
- zachowanie bonding, NVS i listy zapamiętanych urządzeń.

## Połączenie I2S

| Maestro ESP32-S3 | ESP32-WROOM-32U | Funkcja |
|---|---|---|
| GPIO06 | GPIO26 | BCLK |
| GPIO16 | GPIO25 | LRCK / WS |
| GPIO15 | GPIO22 | DATA |
| GND | GND | masa |

## Format audio

- wejście I2S: AUTO 44,1 kHz / 48 kHz, 16-bit, stereo
- wyjście Bluetooth A2DP: 44,1 kHz, 16-bit, stereo

## Pliki

- `src/main.cpp` — główny program ESP32
- `info.txt` — krótki opis programu i połączeń

## Wersja

- `MAESTRO BT V7.4`
- `MRU-3 FULL BT CORE RESTART`
