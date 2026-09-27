use core::future::{self};

use cyw43::{Control, JoinOptions};
use defmt::*;
use embassy_net::Stack;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio_programs::ws2812::PioWs2812;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use embassy_time::{Duration, Ticker, Timer};
use serde::Serialize;
use smart_leds::RGB8;

pub const USER_EVENT_CHANNEL_SIZE: usize = 3;

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

/// Attempts to join the WiFi network and waits for the network stack to come
/// up. Returns `true` once the stack is up, or `false` if all join attempts
/// were exhausted.
pub async fn join_wifi(
    control: &mut Control<'static>,
    stack: Stack<'static>,
    ssid: &'static str,
    password: &'static str,
) -> bool {
    const MAX_JOIN_ATTEMPTS: u32 = 3;
    let mut backoff = Duration::from_secs(2);
    for attempt in 1..=MAX_JOIN_ATTEMPTS {
        match control
            .join(ssid, JoinOptions::new(password.as_bytes()))
            .await
        {
            Ok(()) => {
                info!("WiFi joined on attempt {}", attempt);
                stack.wait_config_up().await;
                info!("Network stack is up");
                return true;
            }
            Err(err) => {
                warn!(
                    "Join attempt {} failed with status: {}",
                    attempt, err.status
                );
                if attempt < MAX_JOIN_ATTEMPTS {
                    Timer::after(backoff).await;
                    backoff *= 2;
                } else {
                    warn!(
                        "Giving up on WiFi after {} attempts; running offline",
                        MAX_JOIN_ATTEMPTS
                    );
                }
            }
        }
    }
    false
}

#[embassy_executor::task]
pub async fn wifi_task(
    mut control: Control<'static>,
    mut state_receiver: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        UserEvent,
        USER_EVENT_CHANNEL_SIZE,
    >,
) {
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

async fn show_initializing_led_strip(ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
    let mut data = [RGB8::default(); NUM_LEDS];
    for i in 0..NUM_LEDS {
        data[i] = RGB8::new(0, 0, 255);
    }
    ws2812.write(&data).await;
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

            ws2812.write(&data).await;
            ticker.next().await;
        }
    }
}

async fn show_stabilizing_led_strip(ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
    let mut data = [RGB8::default(); NUM_LEDS];
    for i in 0..NUM_LEDS {
        data[i] = RGB8::new(255, 255, 0);
    }
    ws2812.write(&data).await;
    future::pending().await
}

async fn show_grinding_led_strip(ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
    let mut data = [RGB8::default(); NUM_LEDS];
    for i in 0..NUM_LEDS {
        data[i] = RGB8::new(0, 255, 0);
    }
    ws2812.write(&data).await;
    future::pending().await
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
            ws2812.write(&data).await;
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}

async fn show_state_led_strip(
    ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>,
    state: UserEvent,
) {
    match state {
        UserEvent::Initializing => show_initializing_led_strip(ws2812).await,
        UserEvent::Idle => show_idle_led_strip(ws2812).await,
        UserEvent::Stabilizing => show_stabilizing_led_strip(ws2812).await,
        UserEvent::Grinding => show_grinding_led_strip(ws2812).await,
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
) {
    let mut state = UserEvent::Initializing;

    loop {
        let result = embassy_futures::select::select(
            state_receiver.changed(),
            show_state_led_strip(&mut ws2812, state),
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
