# RFID-Based Smart Door Lock System

A comprehensive smart door lock system built with ESP32 that combines RFID authentication, PIN verification, and servo motor control for secure access management.

## Features

* **RFID Authentication:** Uses MFRC522 module for card-based access control
* **PIN Verification:** 4x4 matrix keypad for password entry (User: 1234, Admin: 7777)
* **OLED Display:** SSD1306 128x64 display for user interface and feedback
* **Servo Lock Control:** SG90/MG90S servo motor for physical lock mechanism
* **WiFi & NTP Sync:** Automatic time synchronization for access logging
* **EEPROM Storage:** Non-volatile storage for registered RFID cards
* **Finite State Machine:** Efficient state management for system operations
* **Admin Menu:** Add/remove RFID cards and view access history

## Hardware Requirements

| Component | Specification |
| :--- | :--- |
| **Microcontroller** | ESP32 (30-pin or 38-pin variant) |
| **RFID Reader** | MFRC522 (SPI interface) |
| **Display** | SSD1306 OLED 128x64 (I2C interface) |
| **Actuator** | SG90 or MG90S Servo Motor |
| **Input** | 4x4 Matrix Keypad |
| **Storage** | Internal ESP32 EEPROM (Emulated) |

## Pin Assignments

| Component | Pin Name | ESP32 GPIO | Interface |
| :--- | :--- | :--- | :--- |
| **MFRC522** | SS (SDA) | 5 | SPI |
| | SCK | 18 | SPI (Default) |
| | MOSI | 23 | SPI (Default) |
| | MISO | 19 | SPI (Default) |
| | RST | 2 | GPIO |
| **SSD1306** | SCL | 22 | I2C (Default) |
| | SDA | 21 | I2C (Default) |
| **Servo** | PWM | 15 | PWM |
| **Keypad** | Row 1-4 | 13, 12, 14, 27 | GPIO |
| | Col 1-4 | 26, 25, 33, 32 | GPIO |

## Software Dependencies

Install the following libraries via Arduino Library Manager:
* **U8g2** - For SSD1306 OLED rendering
* **ESP32Servo** - For PWM control of the locking mechanism
* **MFRC522** - For RFID card communication
* **Keypad** - For matrix keypad scanning

## Installation

1. Open `Source Code/rfidSmartDoorLockSystem.ino` in Arduino IDE.
2. Select **DOIT ESP32 DEVKIT V1** (or your specific ESP32 board) under **Tools > Board**.
3. Update WiFi credentials in the `setup()` function:
   ```cpp
   WiFi.begin("your_wifi_name", "your_wifi_password");