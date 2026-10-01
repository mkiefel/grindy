use cyw43::{Control, JoinOptions};
use defmt::*;
use embassy_net::{Ipv4Address, Ipv4Cidr, Stack, StaticConfigV4};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex, signal, watch};
use embassy_time::{with_timeout, Duration, Timer};

use crate::storage::{self, SharedFlash, WifiConfig};
use crate::ui::{blink_status_led, UserEvent, USER_EVENT_CHANNEL_SIZE};

/// Access point opened when no WiFi is configured or joining it failed.
const SETUP_AP_SSID: &str = "grindy";
const SETUP_AP_PASSWORD: &str = "grindyrockz";
const SETUP_AP_CHANNEL: u8 = 6;
const SETUP_AP_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 25, 1);

/// Upper bound for a single join attempt. The cyw43 driver waits for the chip
/// to report the join result without a timeout of its own.
const JOIN_TIMEOUT: Duration = Duration::from_secs(20);

/// How long to wait for a DHCP lease after joining a network.
const DHCP_TIMEOUT: Duration = Duration::from_secs(30);

/// Watchdog scratch register that survives the reset in
/// `reboot_into_setup_access_point`. Scratch 4-7 are used by the RP2350
/// bootrom, so use one of the low ones.
const FORCE_SETUP_AP_SCRATCH: usize = 0;
const FORCE_SETUP_AP_MAGIC: u32 = 0x6772_6170; // "grap"

/// Signalled with a new configuration (already stored in flash) to make the
/// network task reconnect.
pub static WIFI_CONFIG_CHANGED: signal::Signal<CriticalSectionRawMutex, WifiConfig> =
    signal::Signal::new();

#[derive(Clone, Copy, PartialEq)]
pub enum WifiMode {
    Connecting,
    Client,
    SetupAccessPoint,
}

pub struct WifiStatus {
    pub mode: WifiMode,
    /// SSID of the configured network, if any.
    pub ssid: Option<heapless::String<{ storage::MAX_SSID_LEN }>>,
}

pub static WIFI_STATUS: mutex::Mutex<CriticalSectionRawMutex, WifiStatus> =
    mutex::Mutex::new(WifiStatus {
        mode: WifiMode::Connecting,
        ssid: None,
    });

async fn set_status(mode: WifiMode, config: Option<&WifiConfig>) {
    let mut status = WIFI_STATUS.lock().await;
    status.mode = mode;
    status.ssid = config.map(|config| config.ssid.clone());
}

/// Attempts to join the WiFi network and waits for a DHCP lease. Returns
/// `true` once the stack is up, or `false` if all join attempts were
/// exhausted.
async fn join_wifi(control: &mut Control<'static>, stack: Stack<'static>, config: &WifiConfig) -> bool {
    const MAX_JOIN_ATTEMPTS: u32 = 3;

    stack.set_config_v4(embassy_net::ConfigV4::Dhcp(Default::default()));

    let options = if config.password.is_empty() {
        JoinOptions::new_open()
    } else {
        JoinOptions::new(config.password.as_bytes())
    };
    let mut backoff = Duration::from_secs(2);
    for attempt in 1..=MAX_JOIN_ATTEMPTS {
        let result = match with_timeout(JOIN_TIMEOUT, control.join(&config.ssid, options.clone())).await {
            Ok(result) => result.map_err(|err| err.status),
            Err(_) => {
                warn!("Join attempt {} timed out", attempt);
                control.leave().await;
                Err(u32::MAX)
            }
        };
        match result {
            Ok(()) => {
                info!("WiFi joined on attempt {}", attempt);
                if with_timeout(DHCP_TIMEOUT, stack.wait_config_up()).await.is_ok() {
                    info!("Network stack is up");
                    return true;
                }
                warn!("No DHCP lease after joining; giving up");
                control.leave().await;
                return false;
            }
            Err(status) => {
                warn!("Join attempt {} failed with status: {}", attempt, status);
                if attempt < MAX_JOIN_ATTEMPTS {
                    Timer::after(backoff).await;
                    backoff *= 2;
                } else {
                    warn!("Giving up on WiFi after {} attempts", MAX_JOIN_ATTEMPTS);
                }
            }
        }
    }
    false
}

async fn start_setup_access_point(control: &mut Control<'static>, stack: Stack<'static>) {
    info!("Starting setup access point {} at 192.168.25.1", SETUP_AP_SSID);
    stack.set_config_v4(embassy_net::ConfigV4::Static(StaticConfigV4 {
        address: Ipv4Cidr::new(SETUP_AP_ADDRESS, 24),
        gateway: None,
        dns_servers: Default::default(),
    }));
    control
        .start_ap_wpa2(SETUP_AP_SSID, SETUP_AP_PASSWORD, SETUP_AP_CHANNEL)
        .await;
}

/// Reboots straight into the setup access point. A failed join leaves the
/// chip configured for WPA3/SAE (auth mode, MFP, supplicant), which
/// `start_ap_wpa2` doesn't reset, so clients are rejected with an encryption
/// mismatch. Starting the AP on a freshly initialised chip avoids that.
fn reboot_into_setup_access_point(watchdog: &mut Watchdog) -> ! {
    warn!("Rebooting into the setup access point");
    watchdog.set_scratch(FORCE_SETUP_AP_SCRATCH, FORCE_SETUP_AP_MAGIC);
    watchdog.trigger_reset();
    loop {
        cortex_m::asm::nop();
    }
}

/// Connects to the WiFi stored in flash, falling back to the setup access
/// point, and reconnects whenever a new configuration is set.
#[embassy_executor::task]
pub async fn network_task(
    mut control: Control<'static>,
    stack: Stack<'static>,
    flash: SharedFlash,
    mut watchdog: Watchdog,
    mut state_receiver: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        UserEvent,
        USER_EVENT_CHANNEL_SIZE,
    >,
) {
    let mut config = flash.lock(|flash| storage::read_wifi_config(&mut flash.borrow_mut()));
    // Consume the flag so that the next reboot tries joining again.
    let mut force_setup_ap = watchdog.get_scratch(FORCE_SETUP_AP_SCRATCH) == FORCE_SETUP_AP_MAGIC;
    watchdog.set_scratch(FORCE_SETUP_AP_SCRATCH, 0);
    loop {
        set_status(WifiMode::Connecting, config.as_ref()).await;
        let connected = match &config {
            Some(config) if !force_setup_ap => {
                if !join_wifi(&mut control, stack, config).await {
                    reboot_into_setup_access_point(&mut watchdog);
                }
                true
            }
            _ => false,
        };
        force_setup_ap = false;
        if connected {
            set_status(WifiMode::Client, config.as_ref()).await;
        } else {
            start_setup_access_point(&mut control, stack).await;
            set_status(WifiMode::SetupAccessPoint, config.as_ref()).await;
        }

        let new_config = match embassy_futures::select::select(
            WIFI_CONFIG_CHANGED.wait(),
            blink_status_led(&mut control, &mut state_receiver),
        )
        .await
        {
            embassy_futures::select::Either::First(new_config) => new_config,
        };
        info!("WiFi config changed to SSID {}; reconnecting", new_config.ssid.as_str());

        // Give the web server a moment to answer the request that changed the
        // config before the connection goes away.
        Timer::after(Duration::from_secs(1)).await;
        if connected {
            control.leave().await;
        } else {
            control.close_ap().await;
        }
        config = Some(new_config);
    }
}
