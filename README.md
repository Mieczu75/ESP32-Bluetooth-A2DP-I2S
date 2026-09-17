# ESP32 Bluetooth A2DP I2S

Nadajnik Bluetooth A2DP dla ESP32-WROOM-32U – MAESTRO BT V7.4

Program odbiera stereofoniczny dźwięk PCM z urządzenia Maestro ESP32-S3 przez magistralę I2S i przesyła go bezprzewodowo do słuchawek lub głośnika Bluetooth w standardzie A2DP.

Obsługuje automatyczne rozpoznawanie wejściowego próbkowania 44,1 kHz i 48 kHz, a transmisja Bluetooth odbywa się z częstotliwością 44,1 kHz. Program zapamiętuje 3 ostatnio używane odbiorniki Bluetooth i próbuje łączyć się z nimi kolejno, zaczynając od ostatnio używanego urządzenia. Jeżeli żaden zapisany odbiornik nie jest dostępny, wykonywane jest krótkie wyszukiwanie nowych urządzeń Audio/Video.

Po ponownym włączeniu wcześniej używanych słuchawek lub głośnika program automatycznie próbuje odtworzyć połączenie. W wersji V7.4 po utracie aktywnego odbiornika wykonywany jest pełny restart warstwy Bluetooth — A2DP, AVRCP, Bluedroid oraz kontrolera Bluetooth — bez restartowania całego ESP32 i bez utraty zapisanych urządzeń oraz danych parowania.

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
