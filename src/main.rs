#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type, core_float_math)]

use cyw43::{JoinOptions, NetDriver};
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{self, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, mutex, watch};
use gpio::{Input, Level, Output};
use picoserve::make_static;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod scale;
mod ui;
mod web;

use crate::scale::{
    controller_task, scale_task, GrinderStateMachine, WeightReading, SCALE_CHANNEL_SIZE,
    WEIGHT_BATCH_CHANNEL_SIZE,
};
use crate::ui::{led_strip_task, led_task, UserEvent, USER_EVENT_CHANNEL_SIZE};
use crate::web::{
    bringup_web_server, websocket_broadcaster_task, WsConnectionRegistry, WEB_TASK_POOL_SIZE,
};

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Grindy - Coffee Grinder Controller"),
    embassy_rp::binary_info::rp_program_description!(
        c"An automated coffee grinder controller using a Raspberry Pi Pico"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn net_task(mut stack: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

fn bringup_network_stack(
    spawner: &Spawner,
    net_device: NetDriver<'static>,
) -> embassy_net::Stack<'static> {
    let (stack, runner) = embassy_net::new(
        net_device,
        embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(core::net::Ipv4Addr::new(192, 168, 1, 9), 24),
            gateway: None,
            dns_servers: Default::default(),
        }),
        make_static!(
            embassy_net::StackResources::<WEB_TASK_POOL_SIZE>,
            embassy_net::StackResources::new()
        ),
        embassy_rp::clocks::RoscRng.next_u64(),
    );
    spawner.must_spawn(net_task(runner));
    stack
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Bring up some UI early to show errors.
    let mut led_strip_pio = Pio::new(p.PIO1, Irqs);
    let program = PioWs2812Program::new(&mut led_strip_pio.common);
    let ws2812 = PioWs2812::new(
        &mut led_strip_pio.common,
        led_strip_pio.sm0,
        p.DMA_CH1,
        p.PIN_1,
        &program,
    );
    static STATE_WATCH: watch::Watch<CriticalSectionRawMutex, UserEvent, USER_EVENT_CHANNEL_SIZE> =
        watch::Watch::new();
    STATE_WATCH.sender().send(UserEvent::Initializing);
    spawner.must_spawn(led_strip_task(ws2812, unwrap!(STATE_WATCH.receiver())));

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.must_spawn(cyw43_task(runner));
    control.init(clm).await;

    let stack = bringup_network_stack(&spawner, net_device);

    while let Err(err) = control
        .join(
            env!("GRINDY_WIFI_SSID"),
            JoinOptions::new(env!("GRINDY_WIFI_PASSWORD").as_bytes()),
        )
        .await
    {
        info!("Join failed with status: {}", err.status)
    }

    info!("Waiting for link...");
    stack.wait_link_up().await;

    info!("Waiting for config...");
    stack.wait_config_up().await;

    info!("Stack is up.");

    let grinder = Output::new(p.PIN_0, Level::High);

    let dt = Input::new(p.PIN_18, Pull::Down);
    let sck = Output::new(p.PIN_19, Level::Low);

    static SCALE_CHANNEL: channel::Channel<CriticalSectionRawMutex, f32, SCALE_CHANNEL_SIZE> =
        channel::Channel::new();
    spawner.must_spawn(scale_task(sck, dt, SCALE_CHANNEL.sender()));

    spawner.must_spawn(led_task(control, unwrap!(STATE_WATCH.receiver())));

    static GRINDER_STATE_MACHINE: StaticCell<
        mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
    > = StaticCell::new();
    let grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine> =
        GRINDER_STATE_MACHINE.init(mutex::Mutex::new(GrinderStateMachine::new(grinder)));

    static WS_REGISTRY: StaticCell<mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>> =
        StaticCell::new();
    let ws_registry = WS_REGISTRY.init(mutex::Mutex::new(WsConnectionRegistry::new()));

    static WEIGHT_BATCH_CHANNEL: channel::Channel<
        CriticalSectionRawMutex,
        heapless::Vec<WeightReading, 4>,
        WEIGHT_BATCH_CHANNEL_SIZE,
    > = channel::Channel::new();

    spawner.must_spawn(websocket_broadcaster_task(
        unwrap!(STATE_WATCH.receiver()),
        WEIGHT_BATCH_CHANNEL.receiver(),
        ws_registry,
    ));

    bringup_web_server(&spawner, stack, &grinder_state_machine, ws_registry);

    info!("Hello, coffee world!");
    controller_task(
        SCALE_CHANNEL.receiver(),
        STATE_WATCH.sender(),
        WEIGHT_BATCH_CHANNEL.sender(),
        &grinder_state_machine,
    )
    .await;
}
