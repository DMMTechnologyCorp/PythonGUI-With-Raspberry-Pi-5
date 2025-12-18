# Multi-Axis Servo Control over Serial (RS-232)

This repository contains a **cross-platform, multi-axis servo control framework** written in Python.  
It enables simultaneous control and monitoring of multiple servo drives connected via **independent serial (COM) ports**, using a **threaded, non-blocking architecture**.

The project is designed as a **practical control and diagnostics layer** for serial-based servo systems rather than a hard real-time motion controller.

---

## Features

### Multi-Axis, Multi-Port Control
- Each servo axis is mapped to its own serial port
- Multiple drives can be controlled concurrently without I/O blocking

### Threaded Communication Model
- Dedicated worker threads per axis for continuous polling
- Thread-safe serial access using a global I/O lock
- Responsive command execution even under high polling rates

### Motion Control Capabilities
- Absolute and relative position moves
- Constant speed commands
- Motion start and stop control
- Encoder-based feedback monitoring

### Parameter Read / Write
- Drive ID configuration
- Gain tuning (P / I / D)
- Gear ratio and limit parameters
- Live readback of position, speed, and torque/current

### Cross-Platform Support
- Runs on **Windows and Linux**
- Uses standard Python serial interfaces
- No OS-specific dependencies

### Optional Graphical Interface
- PySide6-based GUI for interactive control
- Drive discovery and axis selection
- Live status visualization
- Manual motion and parameter tuning tools

---

## Typical Use Cases

- Servo drive commissioning and bench testing
- Multi-axis tuning and diagnostics
- Automation prototyping and lab setups
- Service, maintenance, and field utilities
- Educational or internal engineering tools

This project is **not intended for time-critical interpolation or deterministic motion control**, but rather as a flexible interface for serial-driven servo systems.

---

## Requirements

- Python 3.x  
- `pyserial`  
- `PySide6` (optional, for GUI)

---

## High-Level Architecture

- **Low-level driver module**  
  Handles packet framing, CRC checks, and command execution.

- **Axis worker threads**  
  Each axis runs independently for polling and motion commands.

- **Application layer / GUI**  
  Presents live data and exposes control functionality.

The architecture is modular and designed for extension to additional commands or compatible drive families.

---

## Safety Notice

This software can command real motion hardware.  
Always test in a controlled environment and ensure proper safety interlocks are in place before use.


