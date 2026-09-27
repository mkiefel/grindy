use core::cell::RefCell;

use defmt::*;
use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::{self, raw::CriticalSectionRawMutex};

/// Physical flash size on the Pico 2 (RP2350). `memory.x` only lets the
/// linker place the firmware image in the first 2 MiB of this, leaving the
/// rest free for storage like this.
const FLASH_SIZE: usize = 4 * 1024 * 1024;

/// Offset (from flash start) of the calibration storage sector. Sits right
/// after the 2 MiB reserved for the firmware image in `memory.x`, so it can
/// never be overwritten by a flashed binary.
const CALIBRATION_OFFSET: u32 = 2 * 1024 * 1024;

const MAGIC: u32 = 0x6772_6663; // "grfc"

/// Offset of the WiFi configuration sector, right after the calibration one.
const WIFI_CONFIG_OFFSET: u32 = CALIBRATION_OFFSET + ERASE_SIZE as u32;

const WIFI_MAGIC: u32 = 0x6772_7766; // "grwf"

/// Offset of the target weight sector, right after the WiFi config one.
const TARGET_WEIGHT_OFFSET: u32 = WIFI_CONFIG_OFFSET + ERASE_SIZE as u32;

const TARGET_WEIGHT_MAGIC: u32 = 0x6772_7477; // "grtw"

/// Offset of the lead time sector, right after the target weight one.
const LEAD_TIME_OFFSET: u32 = TARGET_WEIGHT_OFFSET + ERASE_SIZE as u32;

const LEAD_TIME_MAGIC: u32 = 0x6772_7461; // "grta"

pub const MAX_SSID_LEN: usize = 32;
pub const MAX_PASSWORD_LEN: usize = 63;
/// WPA2 passphrases have to be at least this long; an empty password means an
/// open network.
pub const MIN_PASSWORD_LEN: usize = 8;

pub type FlashStorage = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

/// Flash shared between the tasks that persist settings. Flash operations are
/// blocking anyway, so a blocking mutex is good enough.
pub type SharedFlash =
    &'static blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<FlashStorage>>;

#[derive(Clone)]
pub struct WifiConfig {
    pub ssid: heapless::String<MAX_SSID_LEN>,
    pub password: heapless::String<MAX_PASSWORD_LEN>,
}

impl WifiConfig {
    pub fn is_valid(&self) -> bool {
        !self.ssid.is_empty()
            && (self.password.is_empty() || self.password.len() >= MIN_PASSWORD_LEN)
    }
}

/// Reads an `f32` stored with [`write_f32`] at `offset`. Returns `None` if
/// nothing has been stored yet (or the stored data is corrupt).
fn read_f32(flash: &mut FlashStorage, offset: u32, magic: u32) -> Option<f32> {
    let mut buf = [0u8; 8];
    if let Err(err) = flash.blocking_read(offset, &mut buf) {
        warn!("Failed to read flash at {:#x}: {}", offset, err);
        return None;
    }

    if u32::from_le_bytes(buf[0..4].try_into().unwrap()) != magic {
        return None;
    }
    Some(f32::from_le_bytes(buf[4..8].try_into().unwrap()))
}

/// Persists `value` in the sector at `offset` so it can be recovered with
/// [`read_f32`]. Returns `false` if writing failed.
fn write_f32(flash: &mut FlashStorage, offset: u32, magic: u32, value: f32) -> bool {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&magic.to_le_bytes());
    buf[4..8].copy_from_slice(&value.to_le_bytes());

    if let Err(err) = flash.blocking_erase(offset, offset + ERASE_SIZE as u32) {
        warn!("Failed to erase flash sector at {:#x}: {}", offset, err);
        return false;
    }
    if let Err(err) = flash.blocking_write(offset, &buf) {
        warn!("Failed to write flash at {:#x}: {}", offset, err);
        return false;
    }
    true
}

/// Reads the calibration factor previously stored with
/// [`write_calibration_factor`]. Returns `None` if nothing has been stored
/// yet (or the stored data is corrupt).
pub fn read_calibration_factor(flash: &mut FlashStorage) -> Option<f32> {
    let factor = read_f32(flash, CALIBRATION_OFFSET, MAGIC);
    match factor {
        Some(factor) => info!("Loaded calibration factor {} from flash", factor),
        None => info!("No calibration factor stored in flash yet"),
    }
    factor
}

/// Persists `factor` so it can be recovered on the next boot with
/// [`read_calibration_factor`].
pub fn write_calibration_factor(flash: &mut FlashStorage, factor: f32) {
    if write_f32(flash, CALIBRATION_OFFSET, MAGIC, factor) {
        info!("Stored calibration factor {} to flash", factor);
    }
}

/// Reads the target coffee weight previously stored with
/// [`write_target_weight`]. Returns `None` if nothing has been stored yet (or
/// the stored data is corrupt).
pub fn read_target_weight(flash: &mut FlashStorage) -> Option<f32> {
    let weight = read_f32(flash, TARGET_WEIGHT_OFFSET, TARGET_WEIGHT_MAGIC)
        .filter(|weight| weight.is_finite());
    match weight {
        Some(weight) => info!("Loaded target weight {}g from flash", weight),
        None => info!("No target weight stored in flash yet"),
    }
    weight
}

/// Persists `weight` so it can be recovered on the next boot with
/// [`read_target_weight`]. Returns `false` if writing failed.
pub fn write_target_weight(flash: &mut FlashStorage, weight: f32) -> bool {
    let ok = write_f32(flash, TARGET_WEIGHT_OFFSET, TARGET_WEIGHT_MAGIC, weight);
    if ok {
        info!("Stored target weight {}g to flash", weight);
    }
    ok
}

/// Reads the lead time previously stored with [`write_lead_time`]. Returns
/// `None` if nothing has been stored yet (or the stored data is corrupt or
/// out of range).
pub fn read_lead_time(flash: &mut FlashStorage) -> Option<f32> {
    let lead_time = read_f32(flash, LEAD_TIME_OFFSET, LEAD_TIME_MAGIC)
        .filter(|lead_time| (0.0..=grindy_gp::lead_time::MAX_LEAD_TIME).contains(lead_time));
    match lead_time {
        Some(lead_time) => info!("Loaded lead time {}s from flash", lead_time),
        None => info!("No lead time stored in flash yet"),
    }
    lead_time
}

/// Persists `lead_time` so it can be recovered on the next boot with
/// [`read_lead_time`]. Returns `false` if writing failed.
pub fn write_lead_time(flash: &mut FlashStorage, lead_time: f32) -> bool {
    let ok = write_f32(flash, LEAD_TIME_OFFSET, LEAD_TIME_MAGIC, lead_time);
    if ok {
        info!("Stored lead time {}s to flash", lead_time);
    }
    ok
}

/// Reads the WiFi configuration previously stored with [`write_wifi_config`].
/// Returns `None` if nothing has been stored yet (or the stored data is
/// corrupt).
pub fn read_wifi_config(flash: &mut FlashStorage) -> Option<WifiConfig> {
    // Layout: magic (4), ssid length (1), password length (1), ssid bytes,
    // password bytes.
    let mut buf = [0u8; 6 + MAX_SSID_LEN + MAX_PASSWORD_LEN];
    if let Err(err) = flash.blocking_read(WIFI_CONFIG_OFFSET, &mut buf) {
        warn!("Failed to read WiFi config from flash: {}", err);
        return None;
    }

    let magic = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    if magic != WIFI_MAGIC {
        info!("No WiFi config stored in flash yet");
        return None;
    }

    let ssid_len = buf[4] as usize;
    let password_len = buf[5] as usize;
    if ssid_len > MAX_SSID_LEN || password_len > MAX_PASSWORD_LEN {
        warn!("Stored WiFi config is corrupt");
        return None;
    }
    let ssid_start = 6;
    let password_start = ssid_start + ssid_len;
    let ssid = core::str::from_utf8(&buf[ssid_start..password_start]).ok()?;
    let password =
        core::str::from_utf8(&buf[password_start..password_start + password_len]).ok()?;

    let config = WifiConfig {
        ssid: heapless::String::try_from(ssid).ok()?,
        password: heapless::String::try_from(password).ok()?,
    };
    if !config.is_valid() {
        warn!("Stored WiFi config is invalid");
        return None;
    }
    info!("Loaded WiFi config for SSID {} from flash", ssid);
    Some(config)
}

/// Persists `config` so it can be recovered on the next boot with
/// [`read_wifi_config`]. Returns `false` if writing failed.
pub fn write_wifi_config(flash: &mut FlashStorage, config: &WifiConfig) -> bool {
    // Flash writes have to be a multiple of 4 bytes.
    let mut buf = [0u8; (6 + MAX_SSID_LEN + MAX_PASSWORD_LEN).next_multiple_of(4)];
    let ssid = config.ssid.as_bytes();
    let password = config.password.as_bytes();
    buf[0..4].copy_from_slice(&WIFI_MAGIC.to_le_bytes());
    buf[4] = ssid.len() as u8;
    buf[5] = password.len() as u8;
    buf[6..6 + ssid.len()].copy_from_slice(ssid);
    buf[6 + ssid.len()..6 + ssid.len() + password.len()].copy_from_slice(password);

    if let Err(err) =
        flash.blocking_erase(WIFI_CONFIG_OFFSET, WIFI_CONFIG_OFFSET + ERASE_SIZE as u32)
    {
        warn!("Failed to erase WiFi config flash sector: {}", err);
        return false;
    }
    if let Err(err) = flash.blocking_write(WIFI_CONFIG_OFFSET, &buf) {
        warn!("Failed to write WiFi config to flash: {}", err);
        return false;
    }
    info!("Stored WiFi config for SSID {} to flash", config.ssid.as_str());
    true
}
