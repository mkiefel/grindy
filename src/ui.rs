use cyw43::Control;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio_programs::ws2812::PioWs2812;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use embassy_time::{Duration, Ticker, Timer};
use serde::Serialize;
use smart_leds::RGB8;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum UserEvent {
    Idle,
    Stabilizing,
    Grinding,
    WaitingForRemoval,
}

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
    mut state_receiver: watch::Receiver<'static, CriticalSectionRawMutex, UserEvent, 2>,
) {
    loop {
        match state_receiver.get().await {
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

#[embassy_executor::task]
pub async fn led_strip_task(mut ws2812: PioWs2812<'static, PIO1, 0, NUM_LEDS>) {
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
