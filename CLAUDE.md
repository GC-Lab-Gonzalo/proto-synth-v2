# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Proto-Synth v2 is an ESP32-based open-source synthesizer platform by GC Lab Chile. It's a collection of standalone Arduino firmware examples for a custom PCB that combines audio output, sensors, buttons, potentiometers, and LEDs. All documentation and code comments are in **Spanish**.

## Build & Upload

- **IDE:** Arduino IDE with ESP32 board manager installed
- **Board:** ESP32 DevKit v1 (Dual Core, 240MHz)
- **Upload requirement:** Potentiometer 3 (GPIO 12) must be at maximum before uploading (strapping pin issue)
- **Serial Monitor:** 115200 baud (typical), use for debugging
- **Key libraries:** `Mozzi` (advanced audio DSP), `Adafruit_MPU6050` (IMU), `U8g2lib` (OLED display)

There is no automated build system, CI/CD, or test suite. Each firmware is compiled and uploaded individually through Arduino IDE.

## Hardware Pin Map

| Component        | GPIO Pin(s)         |
|-----------------|---------------------|
| DAC Audio Out   | 25 (8-bit, 3.5mm jack) |
| Potentiometers  | 13, 14, 12, 27 (ADC2 — incompatible with WiFi) |
| Buttons         | 18, 4, 15, 19 (pull-up) |
| LEDs            | 23, 32, 5, 2       |
| MPU6050 (I2C)   | SDA=21, SCL=22     |
| LDR             | 26                  |
| Microphone      | 33                  |
| MIDI TX         | TX0 (31250 baud)   |
| Expansion       | 34, 35              |

## Architecture

Each firmware in `firmwares/` is a **standalone `.ino` file** — there is no shared library or common code between examples. They follow this pattern:

1. Pin definitions and constants at top
2. `setup()`: initialize pins, DAC, serial, sensors
3. `loop()`: read inputs → compute audio (oscillators/filters/samples) → write to DAC

**Audio output** uses the ESP32 hardware DAC (`driver/dac.h`) on GPIO 25 at 8-bit resolution. Sample rate is typically 22050 Hz. Some firmwares use timer interrupts for precise sample timing.

**Firmware categories:** synthesis/oscillators, step sequencers, IMU gesture control, samplers/recording, song demos, and one OLED game. See `firmwares/lista.md` for the full catalog.

## Hardware Errata (v2.0)

These affect firmware code patterns:

1. **Inverted potentiometers:** Use `4095 - analogRead(PIN)` to correct reading direction
2. **ADC2/WiFi conflict:** Potentiometers cannot be used simultaneously with the WiFi library
3. **GPIO 12 strapping pin:** POT3 must be at max during upload
4. **Protection diodes removed:** Replaced with solder bridges on assembled boards

## Code Conventions

- Comments and variable names are in Spanish
- Standard Arduino `.ino` format (C++11)
- Musical note frequencies are defined as arrays/constants (e.g., `float notas[]`)
- Waveform generation is typically done with phase accumulators and lookup tables
