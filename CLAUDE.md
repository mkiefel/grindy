# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Grindy is an automated coffee grinder controller for Raspberry Pi Pico 2 (RP2350) written in embedded Rust. It uses a load cell (HX711) to measure coffee weight in real-time and automatically controls grinding to achieve a target weight (18g by default).

## Hardware Platform

- **MCU**: RP2350 (Raspberry Pi Pico 2)
- **Target**: `thumbv8m.main-none-eabihf` (ARMv8-M with hardware FP)
- **Toolchain**: Rust nightly (required)
- **Flash/Debug**: probe-rs with RP235x chip support

## Build and Development Commands

### Building
```bash
cargo build --release
```

### Running/Flashing
```bash
cargo run --release
```
This uses probe-rs to flash and run on the connected Pico 2 via the configured runner in `.cargo/config.toml`.

### Checking Code
```bash
cargo check
```

### Building Debug Version
```bash
cargo build
```
Note: Even release builds include debug symbols (see `profile.release` in Cargo.toml).

## Key Architecture

### Async Runtime
The project uses **Embassy**, an async executor for embedded systems. All major components run as concurrent tasks:

- `main()` spawner orchestrates all tasks
- Tasks communicate via Embassy channels, mutexes, and watch primitives
- Critical section-based synchronization (`CriticalSectionRawMutex`)

### Task Structure

1. **cyw43_task**: Runs the WiFi chip driver (CYW43439) for network connectivity
2. **net_task**: Manages the TCP/IP network stack
3. **web_task** (pool of 8): HTTP server tasks handling status requests
4. **scale_task**: Continuously reads HX711 load cell sensor (~100Hz polling)
5. **led_task**: Controls onboard LED based on grinder state
6. **controller_task**: State machine managing the grinding workflow

### State Machine (GrinderStateMachine)

The controller implements a state machine in `src/main.rs:237-460` with these states:

- **Tare**: Calibrates scale zero point (30 samples)
- **WaitingForCalibration**: Waits for known calibration weight (200g)
- **Calibrating**: Determines scale factor (200 samples)
- **WaitingForPortafilter**: Idle, waiting for portafilter placement
- **Stabilizing**: Ensures weight is stable before grinding (2s window)
- **Grinding**: Active grinding until target weight reached
- **WaitingForRemoval**: Grinding complete, waiting for portafilter removal

State transitions are driven by weight readings from the scale channel.

### Communication Architecture

- **scale_task → controller_task**: `channel::Channel<f32>` (size 5) for raw weight readings
- **controller_task → led_task**: `watch::Watch<UserEvent>` for state broadcasts
- **web tasks**: Access state via shared `Mutex<GrinderStateMachine>`

### Network Configuration

- Static IP: 192.168.22.1/24 (AP mode)
- WiFi AP: SSID "grindy", password "grindyrockz", channel 8
- HTTP endpoint: `GET /` returns current status text

### Hardware Pins

- **Grinder control**: GPIO 0 (active-low relay control)
- **HX711 scale**: GPIO 16 (SCK), GPIO 17 (DT, pull-down)
- **CYW43 WiFi**: PIO0 SPI interface (pins 23-25, 24, 29)

## Memory Layout

The `memory.x` linker script defines RP2350-specific memory regions:
- 2MB Flash at 0x10000000
- 512KB RAM at 0x20000000 (SRAM0-7, striped)
- 4KB SRAM8/9 for dedicated uses

Special sections for RP2350 boot ROM:
- `.start_block`: Boot info block
- `.bi_entries`: Picotool binary info
- `.end_block`: Boot ROM signature

## Logging

Uses `defmt` for efficient embedded logging:
- Log level: `debug` (set in `.cargo/config.toml` via `DEFMT_LOG`)
- Output: RTT (Real-Time Transfer) via `defmt-rtt`
- View logs with probe-rs or other RTT viewer

## Important Constants

Located in `GrinderStateMachine::update_weight()` (src/main.rs:293-305):
- `TARGET_COFFEE_WEIGHT`: 18.0g
- `PORTAFILTER_THRESHOLD`: 100.0g (detection threshold)
- `REMOVAL_THRESHOLD`: 10.0g
- `STABILIZATION_TIME`: 2 seconds
- `WEIGHT_STABILITY_TOLERANCE`: 1.0g
- `MAX_GRIND_TIME_IN_SECS`: 50s (safety timeout)

## Firmware Dependencies

WiFi requires CYW43439 firmware blobs in `cyw43-firmware/`:
- `43439A0.bin`
- `43439A0_clm.bin`

These are loaded at runtime (see `src/main.rs:483-484`).

## Code Organization

Single-file architecture in `src/main.rs`:
- Lines 1-36: Imports and hardware interrupt bindings
- Lines 38-101: Async task definitions
- Lines 103-173: Scale reading and LED control
- Lines 175-222: Network and web server setup
- Lines 224-460: Core state machine logic
- Lines 462-536: Main entry point and initialization

## Development Notes

- No `std` library (`#![no_std]`) - embedded environment
- Requires nightly Rust for `impl_trait_in_assoc_type` feature
- Build script (`build.rs`) handles linker configuration for embedded target
- Uses `make_static!` macro extensively for static allocation (no heap)
