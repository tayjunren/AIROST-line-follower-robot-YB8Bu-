# AIROST IoT Robot — ESP32 Line Following Delivery Robot

AIROST Internship 26/27 · IoT Track · Problem Statement 1: Dual-Mode Autonomous Delivery Robot

## Overview
An ESP32-based dual-mode robot that autonomously follows a black line to a colour-coded destination, picks up a ping pong ball with a servo gripper, then hands control to the user via Bluetooth for the return trip.

## Hardware
| Component | Detail |
|---|---|
| Microcontroller | ESP32 Dev Module |
| Motor Driver | L298N (dual H-bridge) |
| IR Sensors | LM324N 4-sensor array (GPIO 36, 39, 34, 35) |
| Colour Sensor | TCS34725 (I2C: SDA=21, SCL=22) |
| OLED Display | SSD1306 128×64 (I2C: SDA=21, SCL=22) |
| Gripper | Standard servo (GPIO 18) |
| Mode Button | Tactile push button (GPIO 4, INPUT_PULLUP) |

## Features
- **Autonomous line following** with PID controller (Kp=28, Ki=0, Kd=16)
- **Colour detection** via TCS34725 — stops at Red / Blue / Green destination ring
- **Bluetooth manual control** (device name: `AIROST_ESP32_Bot`)
- **OLED status display** — mode, target, detected colour, PID error
- **Servo gripper** — open/close via Bluetooth command

## Bluetooth Commands
| Command | Action |
|---|---|
| `F` / `B` / `L` / `R` | Forward / Backward / Left / Right |
| `S` | Stop |
| `M` | Switch back to AUTO line-follow mode |
| `O` / `C` | Gripper Open / Close |
| `1` / `2` / `3` | Set target destination (Red / Blue / Green) |

## Wiring / Pinout
| Component | ESP32 GPIO |
|---|---|
| I2C SDA (OLED + TCS34725) | GPIO 21 |
| I2C SCL | GPIO 22 |
| L298N ENA (Left PWM) | GPIO 26 |
| L298N IN1 / IN2 | GPIO 27 / 14 |
| L298N ENB (Right PWM) | GPIO 13 |
| L298N IN3 / IN4 | GPIO 16 / 17 |
| IR Far-Left | GPIO 36 |
| IR Mid-Left | GPIO 39 |
| IR Mid-Right | GPIO 34 |
| IR Far-Right | GPIO 35 |
| Servo Signal | GPIO 18 |
| Mode Button | GPIO 4 |

## Setup & Build

### Requirements
- [Arduino IDE 2.x](https://www.arduino.cc/en/software)
- ESP32 board package — add this URL in **File → Preferences → Additional board manager URLs**:
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
  Then install **esp32 by Espressif Systems** via Board Manager.

### Libraries (install via Library Manager)
- `Adafruit TCS34725` by Adafruit
- `Adafruit SSD1306` by Adafruit
- `Adafruit GFX Library` by Adafruit
- `ESP32Servo` by Kevin Harrington
- `BluetoothSerial` — built into the ESP32 Arduino core

### Upload
1. Open `line_follower/line_follower.ino` in Arduino IDE
2. Select **Board → ESP32 Dev Module**
3. Select the correct **COM port**
4. Click **Upload**

## Tuning Notes
- **IR threshold**: sensors use `digitalRead()` with `IR_ACTIVE_LOW = true` — LOW = line detected. Adjust the flag if your modules behave differently.
- **PID constants**: start with `Ki = 0`. Increase `Kp` until the robot tracks without oscillating, then increase `Kd` to dampen overshoot.
- **Colour thresholds**: print serial monitor output while hovering TCS34725 over each coloured ring and adjust the `rNorm / gNorm / bNorm` comparisons in `detectColor()`.
- **Servo angles**: `GRIPPER_OPEN_ANGLE = 30°`, `GRIPPER_CLOSE_ANGLE = 110°` — adjust to your gripper's physical range.
