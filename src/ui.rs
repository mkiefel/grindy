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

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum UserEvent {
    Initializing,
    Idle,
    Stabilizing,
    Grinding,
    WaitingForRemoval,
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

#[embassy_executor::task]
pub async fn led_task(
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
            UserEvent::Stabilizing => {
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

async fn show_waiting_for_removal_led_strip(ws2812: &mut PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
    let mut data = [RGB8::default(); NUM_LEDS];
    loop {
        for j in 0..(256 * 2) {
            let brightness = if j < 256 { j as u8 } else { (511 - j) as u8 };
            for i in 0..NUM_LEDS {
                data[i] = (0, brightness, 0).into();
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
        UserEvent::WaitingForRemoval => show_waiting_for_removal_led_strip(ws2812).await,
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
