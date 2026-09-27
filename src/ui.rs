use core::future::{self};

use cyw43::Control;
use defmt::*;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio_programs::ws2812::PioWs2812;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use embassy_time::{Duration, Ticker, Timer};
use serde::Serialize;
use smart_leds::RGB8;

pub const USER_EVENT_CHANNEL_SIZE: usize = 3;
pub const GRIND_PROGRESS_CHANNEL_SIZE: usize = 1;

/// Receives the grind progress from 0.0 (nothing ground) to 1.0 (target
/// weight reached) while grinding.
pub type GrindProgressReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, f32, GRIND_PROGRESS_CHANNEL_SIZE>;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum UserEvent {
    Initializing,
    Idle,
    Stabilizing,
    Grinding,
    WaitingForRemoval,
    WaitingForCalibration,
    Calibrating,
}

// Generates rainbow colors across 0-255 positions.
//
// This function comes from
// https://github.com/embassy-rs/embassy/blob/39c9f9f26ecb6ef5ce787f5b23809398983414a6/examples/rp/src/bin/pio_ws2812.rs#L23
// and is licensed under MIT OR Apache-2.0.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

/// Blinks the onboard LED (attached to the WiFi chip) according to the
/// current state.
pub async fn blink_status_led(
    control: &mut Control<'static>,
    state_receiver: &mut watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        UserEvent,
        USER_EVENT_CHANNEL_SIZE,
    >,
) -> ! {
    loop {
        match state_receiver.get().await {
            UserEvent::Initializing => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(500)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(500)).await;
            }
            UserEvent::Idle => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(1000)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(1000)).await;
            }
            UserEvent::WaitingForCalibration => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(250)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(750)).await;
            }
            UserEvent::Stabilizing | UserEvent::Calibrating => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(250)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(250)).await;
            }
            UserEvent::Grinding => {
                control.gpio_set(0, true).await;
                state_receiver.changed().await;
            }
            UserEvent::WaitingForRemoval => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

const NUM_LEDS: usize = 6;

/// Upper bound for each color channel of the LED strip. Keeps the current draw
/// low so the strip does not pull the supply down (and with it the WiFi chip).
const MAX_BRIGHTNESS: u8 = 16;

async fn write_led_strip(
    ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>,
    data: &[RGB8; NUM_LEDS],
) {
    let scale = |c: u8| ((c as u16 * MAX_BRIGHTNESS as u16) / 255) as u8;
    let scaled = data.map(|c| RGB8::new(scale(c.r), scale(c.g), scale(c.b)));
    ws2812.write(&scaled).await;
}

async fn show_initializing_led_strip(ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
    let mut data = [RGB8::default(); NUM_LEDS];
    for i in 0..NUM_LEDS {
        data[i] = RGB8::new(0, 0, 255);
    }
    write_led_strip(ws2812, &data).await;
    future::pending().await
}

async fn show_idle_led_strip(ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
    let mut data = [RGB8::default(); NUM_LEDS];
    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        for j in 0..(256 * 5) {
            for i in 0..NUM_LEDS {
                data[i] = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
            }

            write_led_strip(ws2812, &data).await;
            ticker.next().await;
        }
    }
}

async fn show_stabilizing_led_strip(ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
    let mut data = [RGB8::default(); NUM_LEDS];
    for i in 0..NUM_LEDS {
        data[i] = RGB8::new(255, 255, 0);
    }
    write_led_strip(ws2812, &data).await;
    future::pending().await
}

/// How far (in LEDs) the progress has to fall below an LED's switch-on point
/// before it is switched off again. Keeps scale noise from making the edge of
/// the bar flicker.
const PROGRESS_HYSTERESIS: f32 = 0.25;

/// Returns the number of LEDs to light for `progress` (0.0 to 1.0), given that
/// `lit` LEDs are currently on. LED `n` (1-based) switches on once the progress
/// reaches `n / NUM_LEDS`, and only switches off again once it drops below that
/// by more than `PROGRESS_HYSTERESIS`.
fn lit_leds(progress: f32, mut lit: usize) -> usize {
    let level = progress.clamp(0.0, 1.0) * NUM_LEDS as f32;
    while lit < NUM_LEDS && level >= (lit + 1) as f32 {
        lit += 1;
    }
    while lit > 0 && level < lit as f32 - PROGRESS_HYSTERESIS {
        lit -= 1;
    }
    lit
}

/// Lights the first `lit` LEDs of the strip green.
fn progress_bar(lit: usize) -> [RGB8; NUM_LEDS] {
    core::array::from_fn(|i| {
        if i < lit {
            RGB8::new(0, 255, 0)
        } else {
            RGB8::default()
        }
    })
}

async fn show_grinding_led_strip(
    ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>,
    progress_receiver: &mut GrindProgressReceiver,
) {
    let mut lit = 0;
    write_led_strip(ws2812, &progress_bar(lit)).await;
    loop {
        let progress = progress_receiver.changed().await;
        let new_lit = lit_leds(progress, lit);
        if new_lit != lit {
            lit = new_lit;
            write_led_strip(ws2812, &progress_bar(lit)).await;
        }
    }
}

async fn show_pulsing_led_strip(
    ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>,
    color: impl Fn(u8) -> RGB8,
) {
    let mut data = [RGB8::default(); NUM_LEDS];
    loop {
        for j in 0..(256 * 2) {
            let brightness = if j < 256 { j as u8 } else { (511 - j) as u8 };
            for i in 0..NUM_LEDS {
                data[i] = color(brightness);
            }
            write_led_strip(ws2812, &data).await;
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}

async fn show_state_led_strip(
    ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>,
    progress_receiver: &mut GrindProgressReceiver,
    state: UserEvent,
) {
    match state {
        UserEvent::Initializing => show_initializing_led_strip(ws2812).await,
        UserEvent::Idle => show_idle_led_strip(ws2812).await,
        UserEvent::Stabilizing => show_stabilizing_led_strip(ws2812).await,
        UserEvent::Grinding => show_grinding_led_strip(ws2812, progress_receiver).await,
        UserEvent::WaitingForRemoval => {
            show_pulsing_led_strip(ws2812, |brightness| (0, brightness, 0).into()).await
        }
        UserEvent::WaitingForCalibration => {
            show_pulsing_led_strip(ws2812, |brightness| (0, 0, brightness).into()).await
        }
        UserEvent::Calibrating => show_stabilizing_led_strip(ws2812).await,
    }
}

#[embassy_executor::task]
pub async fn led_strip_task(
    mut ws2812: PioWs2812<'static, PIO1, 0, NUM_LEDS>,
    mut state_receiver: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        UserEvent,
        USER_EVENT_CHANNEL_SIZE,
    >,
    mut progress_receiver: GrindProgressReceiver,
) {
    let mut state = UserEvent::Initializing;

    loop {
        let result = embassy_futures::select::select(
            state_receiver.changed(),
            show_state_led_strip(&mut ws2812, &mut progress_receiver, state),
        )
        .await;

        match result {
            embassy_futures::select::Either::First(new_state) => {
                state = new_state;
            }
            embassy_futures::select::Either::Second(()) => {
                warn!("LED strip task ended unexpectedly");
            }
        }
    }
}
