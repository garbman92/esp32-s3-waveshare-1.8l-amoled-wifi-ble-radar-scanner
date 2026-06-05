# OmegaX Wireless Field Scanner Firmware

Firmware for the **OmegaX Wireless Field Scanner** running on the **Waveshare ESP32-S3 Touch AMOLED 1.8** board.

This project turns the ESP32-S3 AMOLED board into a portable wireless field scanner with:

* Wi-Fi scanning
* BLE scanning
* Wi-Fi/BLE detail screens
* Live signal tracking
* Mode-based live radar
* Wi-Fi-only radar mode
* BLE-only radar mode
* Touchscreen UI
* OmegaX sci-fi field scanner interface

The Waveshare ESP32-S3 Touch AMOLED 1.8 supports Arduino IDE development, which is what this firmware uses. The board uses a 368 × 448 AMOLED display and the included `pin_config.h` defines the display and touch pins for this project.

---

## Required Files

Your Arduino sketch folder should contain these files:

```text
OmegaX_Wireless_Field_Scanner_v1_3_ModeRadar/
├── OmegaX_Wireless_Field_Scanner_v1_3_ModeRadar.ino
└── pin_config.h
```

Important: the folder name should match the `.ino` file name.

Example:

```text
OmegaX_Wireless_Field_Scanner_v1_3_ModeRadar
```

Inside that folder, place:

```text
OmegaX_Wireless_Field_Scanner_v1_3_ModeRadar.ino
pin_config.h
```

---

## Hardware

Recommended board:

```text
Waveshare ESP32-S3-Touch-AMOLED-1.8
```

This firmware is built around the Waveshare AMOLED 1.8 pin setup:

```cpp
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK  11
#define LCD_CS    12

#define LCD_WIDTH  368
#define LCD_HEIGHT 448

#define IIC_SDA 15
#define IIC_SCL 14
#define TP_INT  21
```

---

## Arduino IDE Setup

### 1. Install Arduino IDE

Download and install Arduino IDE 2.x.

---

### 2. Add ESP32 Board Support

Open Arduino IDE.

Go to:

```text
File > Preferences
```

In **Additional Boards Manager URLs**, add:

```text
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Then go to:

```text
Tools > Board > Boards Manager
```

Search for:

```text
esp32
```

Install:

```text
esp32 by Espressif Systems
```

---

## Board Settings

In Arduino IDE, go to:

```text
Tools > Board
```

Select:

```text
ESP32S3 Dev Module
```

Recommended settings:

```text
Board: ESP32S3 Dev Module
USB CDC On Boot: Enabled
CPU Frequency: 240MHz
Core Debug Level: None
USB DFU On Boot: Disabled
USB Firmware MSC On Boot: Disabled
Upload Mode: UART0 / Hardware CDC
Upload Speed: 921600 or 460800
Flash Mode: QIO
Flash Frequency: 80MHz
Partition Scheme: Huge APP or Default 4MB with spiffs
PSRAM: OPI PSRAM / Enabled, if available
```

If upload fails, lower upload speed to:

```text
460800
```

or:

```text
115200
```

---

## Required Libraries

Install these libraries in Arduino IDE.

Go to:

```text
Sketch > Include Library > Manage Libraries
```

Install:

```text
Arduino_GFX_Library
```

The project also uses the ESP32 built-in libraries:

```cpp
#include <WiFi.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
```

These come with the ESP32 board package.

You also need the Waveshare drive bus library:

```cpp
#include "Arduino_DriveBus_Library.h"
```

If Arduino cannot find this library, download the official Waveshare Arduino demo/resource package for your board and copy the required libraries into your Arduino libraries folder.

Typical Arduino libraries folder:

Windows:

```text
Documents\Arduino\libraries
```

macOS:

```text
~/Documents/Arduino/libraries
```

Linux:

```text
~/Arduino/libraries
```

After copying libraries, restart Arduino IDE.

---

## Installing the Firmware

### Option 1: Download ZIP from GitHub

Click:

```text
Code > Download ZIP
```

Extract the ZIP.

Open the folder:

```text
OmegaX_Wireless_Field_Scanner_v1_3_ModeRadar
```

Double-click:

```text
OmegaX_Wireless_Field_Scanner_v1_3_ModeRadar.ino
```

Arduino IDE should open the project.

---

### Option 2: Clone with Git

Run:

```bash
git clone https://github.com/YOUR-USERNAME/YOUR-REPO-NAME.git
cd YOUR-REPO-NAME
```

Open the `.ino` file in Arduino IDE.

---

## Uploading to the Board

1. Connect the Waveshare ESP32-S3 Touch AMOLED board to your computer with USB.
2. In Arduino IDE, choose the correct port:

```text
Tools > Port
```

3. Click **Verify** to compile.
4. Click **Upload**.

If upload does not start, hold the **BOOT** button, click Upload, then release BOOT when the upload begins.

---

## Using the Scanner

After flashing, the device will boot into the OmegaX Wireless Field Scanner UI.

Main features:

### Wi-Fi Scanner

Scans nearby Wi-Fi networks and displays signal information.

### BLE Scanner

Scans nearby Bluetooth Low Energy devices.

### Live Radar

The radar is mode-based.

Press:

```text
SCAN WIFI
```

to show only Wi-Fi targets.

Press:

```text
SCAN BLE
```

to show only BLE targets.

The radar dots move based on signal strength:

```text
Stronger RSSI = closer to center
Weaker RSSI = farther from center
```

This is not true physical distance. It is a signal-proximity radar. Moving closer to a router, phone, headphones, or BLE beacon should usually move the dot closer to the center.

---

## Troubleshooting

### Arduino says `Arduino_GFX_Library.h` not found

Install `Arduino_GFX_Library` from Library Manager.

---

### Arduino says `Arduino_DriveBus_Library.h` not found

Download the Waveshare demo/resource package for the ESP32-S3 Touch AMOLED 1.8 and copy the included Arduino libraries into:

```text
Documents/Arduino/libraries
```

Then restart Arduino IDE.

---

### Upload fails

Try these fixes:

```text
Use a data USB cable, not a charge-only cable.
Lower upload speed to 460800 or 115200.
Hold BOOT while upload starts.
Try a different USB port.
Close Serial Monitor before uploading.
```

---

### Screen stays black

Check that:

```text
pin_config.h is in the same folder as the .ino file.
The correct board is selected.
The required Waveshare libraries are installed.
The project is for the ESP32-S3 Touch AMOLED 1.8, not another Waveshare display.
```

---

### BLE scan is slow

BLE scans take longer than Wi-Fi scans. The radar uses timed scan cycles so the screen does not completely freeze during scanning.

---

### Wi-Fi and BLE are not shown together

This is intentional. The radar is designed to show one mode at a time:

```text
Wi-Fi radar = Wi-Fi targets only
BLE radar = BLE targets only
```

This keeps the radar readable and prevents too much clutter on the small screen.

---

## Project Notes

The ESP32-S3 cannot measure exact distance using Wi-Fi or BLE alone. This firmware uses RSSI signal strength as a proximity estimate.

General rule:

```text
-35 dBm to -50 dBm = very close / strong
-50 dBm to -65 dBm = nearby
-65 dBm to -80 dBm = farther away
Below -80 dBm = weak / far / unstable
```

Walls, people, metal objects, batteries, antennas, and device orientation can all affect signal strength.

---

## Current Version

```text
OmegaX Wireless Field Scanner v1.3 Mode Radar
```

Major v1.3 radar change:

```text
SCAN WIFI = Wi-Fi-only live radar
SCAN BLE = BLE-only live radar
```

No mixed Wi-Fi/BLE clutter on the same radar screen.

