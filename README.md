# Megasquirt Drive By Wire
This repo contains source code and PCB design files for a Megasquirt Extended CAN device that controls a drive by wire throttle body.

![system_block_diagram](/doc/system_block_diagram.svg)

## 🚧 Project Status Disclaimer

**USE AT YOUR OWN RISK!!!**  
This project is a **work in progress** and is **not yet fully validated**.  
I have **not installed or used this board in my own vehicle**, so any real-world behavior has **not** been tested. All testing so far has been limited to a test bench.

Please also read the **[Safety & Liability Disclaimer](#%EF%B8%8F-safety--liability-disclaimer)**.

## ✨ Current Features
- Tunable from Tuner Studio (via Megasquirt CAN passthrough protocol)
- Data logging support
- Built-in safety features:
  - Dual-sensor redundancy (compares PPS & TPS sensor pairs)
  - Motor driver over-current protection
  - Motor driver over-temperature protection
- CPU watchdog support
  - Enable via `WATCHDOG_SUPPORT` in [config.h](/src/arduino/EThrottle/config.h)
  - Requires the [mcp-can-boot](https://github.com/crycode-de/mcp-can-boot) bootloader, as the default Arduino bootloader does not correctly support watchdog resets

## 🛠️ Future Features
- **v2 PCB** with improved power supply protection (see issue #44)
  - Temporary fixes include added filter capactors on 12v and 5v rails and replacing the Arduino Nano's 5v regulator with a 7805. Without these modifications, the original regulator could fail during aggresive throttle movement, resulting in frying the MCU and CAN transceiver
- Cruise control functionality
- Support throttle "blip" requests (e.g. for automatic/sequential transmissions)

## Building The Firmware
The firmware is a [PlatformIO](https://platformio.org/) project located in [/src/arduino/EThrottle/](/src/arduino/EThrottle/). Library dependencies are declared in its [platformio.ini](/src/arduino/EThrottle/platformio.ini) and downloaded automatically by PlatformIO.

1. Install PlatformIO (either the [VS Code extension](https://platformio.org/install/ide?install=vscode) or [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html))

2. Build the firmware
`pio run -d src/arduino/EThrottle`

3. Flash the firmware to the Arduino Nano
`pio run -d src/arduino/EThrottle -t upload`

When using VS Code, open the `src/arduino/EThrottle/` folder as a PlatformIO project.

### Unit Tests
Hardware independent code is unit tested on the host using PlatformIO's `native` platform (requires a host `gcc`/`g++`). Tests live in [/src/arduino/EThrottle/test/](/src/arduino/EThrottle/test/).

`pio test -d src/arduino/EThrottle -e native`

## Printed Circuit Board
The project files for the PCB are located in [/src/pcb/](/src/pcb/). They can be opened with [EasyEDA Pro](https://easyeda.com/). __Note that they will not open with EasyEDA Standard version__ (might convert them to std version later).

Here's a render of the printed circuit board (PCB).

![pcb_render](/doc/board_layout/pcb_v1.0.png)

The board is designed to be installed into the prototype area of the Megasquirt DIYPNP via a 20pin header connection.

![pcb_installed](/doc/board_layout/pcb_v1.0_installed.jpg)

## ⚠️ Safety & Liability Disclaimer

**This project implements a drive-by-wire throttle control system.  
It is a safety-critical automotive component.**

Use of this code, hardware design, or any related materials is **entirely at your own risk**.

- This project is provided **“as is”**, with **no warranties** of any kind.  
- No guarantee is made regarding safety, correctness, or fitness for any purpose.  
- **I am not liable** for any damages, malfunctions, failures, injuries, or losses that result from using, modifying, or relying on this project.

**Do not use this in any real vehicle** without proper engineering validation, redundancy, testing, and compliance with all relevant safety standards and regulations.