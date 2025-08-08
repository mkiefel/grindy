#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{self, Pull};
use embassy_time::Delay;
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
    let dt = Input::new(p.PIN_17, Pull::Down);
    let sck = Output::new(p.PIN_16, Level::Low);
    let delay = Delay {};

    let mut scale = loadcell::hx711::HX711::new(sck, dt, delay);
    scale.set_gain_mode(GainMode::A64);

    // The initial tare seems to be a bit off. Just throw it away.
    scale.tare(10);
    scale.tare(20);
    scale.set_scale(1.0);
    // Get some reference weight and adjust the scale, with something like <reference_weight> /
    // <values that you get with scaling 1>. E.g.,
    // scale.set_scale(403.0 / 180919.2);

    let mut buffer = heapless::HistoryBuffer::<_, 25>::new();

    loop {
        if scale.is_ready() {
            let reading = scale.read_scaled().unwrap();
            buffer.write(reading);

            info!(
                "Weight {}",
                ((buffer.iter().sum::<f32>() / (buffer.len() as f32)) * 10.0).floor() / 10.0
            );
        }
    }
}
