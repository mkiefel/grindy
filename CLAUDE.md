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

### Testing
```bash
cargo test-gp -- --nocapture
```
Runs the host tests of the `grindy-gp` crate (a cargo alias for `cargo test
-p grindy-gp --target host-tuple`, since `.cargo/config.toml` pins the
default build target to the MCU): the GP estimator's invariants, its
equivalence with a batch-GP reference, and a replay regression over the logs
in `logs/`.

```bash
cd logger && uv run --with pytest pytest -q
```
Runs the logger's Python tests.

After adding new grinds to `logs/`, refit the estimator's hyperparameters and
regenerate the test fixtures:
```bash
uv run tools/gp/fit.py && uv run tools/gp/export_fixtures.py
```

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

The controller implements a state machine in `src/scale.rs` with these states:

- **Tare**: Calibrates scale zero point (30 samples)
- **WaitingForCalibration**: Waits for known calibration weight (200g)
- **Calibrating**: Determines scale factor (200 samples)
- **WaitingForPortafilter**: Idle, waiting for portafilter placement
- **Stabilizing**: Ensures weight is stable before grinding (2s window)
- **Grinding**: Active grinding until the grind estimator predicts the target,
  the raw weight reaches it, or the safety timeout fires
- **WaitingForRemoval**: Grinding complete; measures the settled weight and
  learns the lead time, then waits for portafilter removal

State transitions are driven by weight readings from the scale channel.

In **Grinding**, readings within `RAMP_UP` (1s) of the grind's start don't
feed the estimator (the flow is still ramping up). Once fed, the grind stops
at the first of: the estimator's forecast reaches the target after
`lead_time` more seconds (`StopReason::Prediction`), the raw coffee weight
reaches the target (`StopReason::RawWeight`, kept as a safety net), or
`MAX_GRIND_TIME_IN_SECS` elapses (`StopReason::Timeout`).

In **WaitingForRemoval**, readings from 1.5s to 2.5s after the stop (the
settle window) are averaged (MAD-filtered) into the settled weight. If the
stop wasn't a timeout, the window was stable, and the observed lead time is
plausible, `lead_time` is updated by an EMA (rate 0.2) and, if the change
exceeds 0.01s, persisted to flash. Portafilter removal moves on to
`WaitingForPortafilter` at any time, even mid-settle.

### Communication Architecture

- **scale_task → controller_task**: `channel::Channel<ScaleSample>` (size 5), where `ScaleSample = (Instant, f32)` is a raw weight reading timestamped at the HX711 read
- **controller_task → led_task**: `watch::Watch<UserEvent>` for state broadcasts
- **controller_task → led_strip_task**: `watch::Watch<f32>` grind progress (0..1, coffee/target weight, using the GP-filtered weight once available) while grinding; the strip switches green LEDs on one by one (with hysteresis against flicker)
- **controller_task → websocket_broadcaster_task**: `channel::Channel<ControllerEvent>` (size 4); `ControllerEvent` is `Reading(WeightReading)` (one per scale sample) or `GrindFinished(GrindFinished)` (once per grind, once the settled weight has been measured or the grind was abandoned)
- **websocket_broadcaster_task / web handlers → WebSocket clients**: `WsConnectionRegistry` broadcasts each `WsMessage` into a per-connection queue (`WS_QUEUES`, 16 messages each); a client that falls that far behind has messages dropped
- **web tasks**: Access state via shared `Mutex<GrinderStateMachine>`

### Grind estimator

`grindy-gp/` is a `no_std` workspace crate (depends only on `libm`) providing
a Gaussian-process estimate of the coffee weight over time: the flow rate is
modelled as an Ornstein-Uhlenbeck process around an unknown mean rate, the
weight is its integral, and readings are noisy. Because the kernel is
Markov, inference is exact and O(1) per sample via a Kalman filter on
`[weight, rate, mean rate]`. It provides the stop rule
(`forecast(now + lead_time).mean ≥ target`) and the lead-time learning used
above. Hyperparameters are fitted offline from `logs/` and compiled into the
generated `grindy-gp/src/fitted.rs` (`FITTED: Params`, and
`grindy_gp::DEFAULT_LEAD_TIME`, exported from the crate root); the mean rate
is estimated online per grind. See
`docs/superpowers/specs/2026-09-27-gp-grind-estimator-design.md` for the
full model and the fitting/tooling details.

### Network Configuration

See `src/wifi.rs`. WiFi credentials are stored in flash (sector after the
calibration sector, see `src/storage.rs`) and set via `POST /wifi` from the web page.

- On boot, joins the stored network using DHCP.
- If nothing is stored or joining fails: opens the setup AP "grindy" (password
  "grindyrockz", channel 6) with static IP 192.168.25.1/24 (no DHCP server;
  clients need a static IP in 192.168.25.0/24).
- Saving new credentials stores them and makes `network_task` reconnect.
- `GET /wifi` returns the current mode and SSID as JSON.

Flash also stores the calibration factor, the target weight, and the
learned lead time, each in its own sector (`src/storage.rs`); the lead-time
sector follows the target-weight sector (magic `0x6772_7461`, "grta").

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

In `src/scale.rs` (`GrinderStateMachine` and its `update_weight()`):
- Target coffee weight: 18.0g default (`DEFAULT_TARGET_WEIGHT`), configurable 1-100g via
  `POST /target-weight` from the web page and stored in flash (sector after the WiFi one)
- `PORTAFILTER_THRESHOLD`: 100.0g (detection threshold)
- `REMOVAL_THRESHOLD`: 10.0g
- Stabilizing window: `SAMPLE_COUNT` = 15 samples (~2s at ~100Hz), stable within
  `stability_threshold()` (3 sd of the scale noise) of their mean
- `MAX_GRIND_TIME_IN_SECS`: 50s (safety timeout)
- Settle window (`SETTLE_START`/`SETTLE_END`): 1.5-2.5s after the grind stops,
  used to measure the settled weight and learn the lead time

In `grindy-gp` (`lib.rs`, `lead_time.rs`, `fitted.rs`):
- `RAMP_UP`: 1.0s, readings within this of the grind's start don't feed the estimator
- `MIN_UPDATES_FOR_STOP`: 5, minimum estimator updates before it may stop the grinder
- `MAX_LEAD_TIME`: 2.0s, longest plausible lead-time observation; longer ones are rejected
- `DEFAULT_LEAD_TIME`: fitted (currently ≈0.49s), used until a lead time is learned and stored in flash; see `fitted.rs`

## Firmware Dependencies

WiFi requires CYW43439 firmware blobs in `cyw43-firmware/`:
- `43439A0.bin`
- `43439A0_clm.bin`

These are loaded at runtime in `main()` in `src/main.rs`.

## Code Organization

- `src/main.rs`: hardware/task bring-up and the `main()` entry point
- `src/scale.rs`: HX711 reading (`scale_task`), the `GrinderStateMachine` state
  machine and `controller_task`
- `src/web.rs`: HTTP/WebSocket server, `WsMessage` wire format, broadcaster task
- `src/ui.rs`: LED (status) and LED strip (grind progress) tasks
- `src/storage.rs`: flash-backed calibration factor, target weight, WiFi
  credentials and lead time
- `src/wifi.rs`: WiFi join/AP-fallback logic (`network_task`)
- `grindy-gp/`: `no_std` grind estimator crate (GP/Kalman filter, lead-time logic, fitted constants)
- `tools/gp/`: Python fitting and fixture-export scripts for `grindy-gp`
- `logger/`: standalone Python tool that records a live grind's WebSocket stream to Parquet

## Development Notes

- No `std` library (`#![no_std]`) - embedded environment
- Requires nightly Rust for `impl_trait_in_assoc_type` feature
- Build script (`build.rs`) handles linker configuration for embedded target
- Uses `make_static!` macro extensively for static allocation (no heap)
