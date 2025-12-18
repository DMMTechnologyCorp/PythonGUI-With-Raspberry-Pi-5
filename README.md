# Multi-Axis Servo Control over Serial (RS-232)

This repository contains a **cross-platform, multi-axis servo control application** written in Python.  
It provides a practical framework for controlling and monitoring multiple servo drives connected via **independent serial (COM) ports**, using a **threaded, non-blocking communication model**.

The project is intended as a **control and diagnostics utility** for serial-based servo systems and is not designed to function as a hard real-time motion controller.

---

## YouTube Video for Demonstration and Overview
*A demonstration and walkthrough video will be linked here once available.*

(YouTube link to be updated)

---

## Features

- **Multi-Axis Control**  
  Control multiple servo axes simultaneously, each mapped to its own serial port.

- **Threaded Serial Communication**  
  Dedicated worker threads per axis allow continuous polling and responsive command execution without blocking I/O.

- **Motion Commands**  
  - Absolute and relative positioning  
  - Constant speed control  
  - Motion start and stop commands  

- **Parameter Read / Write**  
  - Drive ID configuration  
  - Gain tuning (P / I / D)  
  - Gear ratio and limit parameters  
  - Live feedback for position, speed, and torque/current  

- **Optional Graphical User Interface**  
  - PySide6-based GUI  
  - Drive discovery and axis selection  
  - Live status visualization  
  - Manual motion and parameter tuning tools  

- **Cross-Platform Support**  
  Runs on **Windows and Linux** using standard Python serial libraries.

---

## Requirements

- Python 3.x  
- `pyserial`  
- `PySide6` (optional, for GUI usage)  
- Compatible servo drives supporting RS-232 communication  

---

## Getting Started

### Basic Setup

1. Connect each servo drive to the host system using RS-232 (one drive per serial port).
2. Power up the servo drives and motors.
3. Run the Python application or launch the provided executable (if available).
4. Select the detected drives and begin issuing motion or parameter commands.

---

## Code Purpose

The purpose of this repository is to provide a **simple, extensible starting point** for users interested in controlling serial-based servo drives from a PC environment.

This project is well suited for:
- Bench testing and commissioning
- Multi-axis diagnostics and tuning
- Automation prototyping and lab setups
- Service, maintenance, and internal engineering tools

Users are encouraged to expand upon this framework to suit their specific motion control or automation needs.

---

## Resources

YouTube Channel:  
* https://www.youtube.com/@dmmtechnologycorp.1610  

Website / Catalogue / Sales Contact:  
* https://www.dmm-tech.com/

---

## Safety Notice

This software can command real motion hardware.  
Always test in a controlled environment and ensure appropriate safety interlocks are in place before use.
