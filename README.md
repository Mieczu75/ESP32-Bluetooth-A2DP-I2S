# ESP32 Bluetooth A2DP I2S

Nadajnik Bluetooth A2DP oparty na ESP32-WROOM-32U. Program odbiera dźwięk PCM z urządzenia Maestro przez I2S i przesyła go do głośnika Bluetooth.

## Najważniejsze funkcje

- ESP32-WROOM-32U jako Bluetooth Classic / A2DP Source
- I2S RX w trybie SLAVE, 16-bit stereo
- automatyczne rozpoznawanie wejścia 44,1 kHz lub 48 kHz
- 44,1 kHz: przekazywanie PCM bez resamplingu
- 48 kHz: automatyczny resampling 48 kHz → 44,1 kHz
- automatyczne czyszczenie bufora przy zmianie częstotliwości
- dźwięk powitalny po zestawieniu połączenia audio
- ponowne łączenie z ostatnim głośnikiem oraz automatyczne parowanie
- diagnostyka przez Serial: częstotliwość, bufor, DROP i UNDERRUN

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

`MAESTRO BT / I2S AUTO 44.1/48K -> A2DP V6`
