#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{self, Pull};
use embassy_time::Delay;
use embassy_time::Timer;
use gpio::{Input, Level, Output};
use loadcell::{hx711::GainMode, LoadCell};
use num_traits::float::FloatCore;
use {defmt_rtt as _, panic_probe as _};

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Grindy"),
    embassy_rp::binary_info::rp_program_description!(
        c"Runs a grinder by weight."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Grinder is controlled with a n-channel MOSFET on GPIO 0.
    let mut grinder = Output::new(p.PIN_0, Level::Low);

    info!("Hello, coffee world!");
    let mut counter = 0;
    while counter < 10 {
        info!("Starting in {}...", 10 - counter);
        counter += 1;
        Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
    info!("Starting grind!");
    grinder.set_high();

    let dt = Input::new(p.PIN_17, Pull::Down);
    let sck = Output::new(p.PIN_16, Level::Low);
    let delay = Delay {};

    let mut scale = loadcell::hx711::HX711::new(sck, dt, delay);
    scale.set_gain_mode(GainMode::A64);

    // The initial tare seems to be a bit off. Just throw it away.
    scale.tare(10);
    scale.tare(20);
    // scale.set_scale(1.0);
    // Get some reference weight and adjust the scale, with something like <reference_weight> /
    // <values that you get with scaling 1>. E.g.,
    scale.set_scale(403.0 / 180919.2);

    let mut buffer = heapless::HistoryBuffer::<_, 8>::new();

    loop {
        if scale.is_ready() {
            let reading = scale.read_scaled().unwrap();
            buffer.write(reading);

            let avg_weight = (buffer.iter().sum::<f32>() / (buffer.len() as f32)).abs();

            info!("Weight {}", (avg_weight * 10.0).floor() / 10.0);

            if avg_weight >= 18.0 {
                info!("Done!");
                grinder.set_low();
                break;
            }
        }
    }

    loop {}
}
