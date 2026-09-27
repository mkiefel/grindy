use defmt::*;
use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

/// Physical flash size on the Pico 2 (RP2350). `memory.x` only lets the
/// linker place the firmware image in the first 2 MiB of this, leaving the
/// rest free for storage like this.
const FLASH_SIZE: usize = 4 * 1024 * 1024;

/// Offset (from flash start) of the calibration storage sector. Sits right
/// after the 2 MiB reserved for the firmware image in `memory.x`, so it can
/// never be overwritten by a flashed binary.
const CALIBRATION_OFFSET: u32 = 2 * 1024 * 1024;

const MAGIC: u32 = 0x6772_6663; // "grfc"

pub type FlashStorage = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

/// Reads the calibration factor previously stored with
/// [`write_calibration_factor`]. Returns `None` if nothing has been stored
/// yet (or the stored data is corrupt).
pub fn read_calibration_factor(flash: &mut FlashStorage) -> Option<f32> {
    let mut buf = [0u8; 8];
    if let Err(err) = flash.blocking_read(CALIBRATION_OFFSET, &mut buf) {
        warn!("Failed to read calibration factor from flash: {}", err);
        return None;
    }

    let magic = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    if magic != MAGIC {
        info!("No calibration factor stored in flash yet");
        return None;
    }

    let factor = f32::from_le_bytes(buf[4..8].try_into().unwrap());
    info!("Loaded calibration factor {} from flash", factor);
    Some(factor)
}

/// Persists `factor` so it can be recovered on the next boot with
/// [`read_calibration_factor`].
pub fn write_calibration_factor(flash: &mut FlashStorage, factor: f32) {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    buf[4..8].copy_from_slice(&factor.to_le_bytes());

    if let Err(err) = flash.blocking_erase(CALIBRATION_OFFSET, CALIBRATION_OFFSET + ERASE_SIZE as u32)
    {
        warn!("Failed to erase calibration flash sector: {}", err);
        return;
    }
    if let Err(err) = flash.blocking_write(CALIBRATION_OFFSET, &buf) {
        warn!("Failed to write calibration factor to flash: {}", err);
    } else {
        info!("Stored calibration factor {} to flash", factor);
    }
}
