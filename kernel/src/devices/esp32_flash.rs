// Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! ESP32-C3 on-chip flash loadable-image Misc device.

use crate::devices::{Device, DeviceClass, DeviceId, DeviceManager};
use crate::drivers::flash::internal_flash::{
    with_internal_flash, EspFlashError, ESP_FLASH_SECTOR_SIZE, ESP_FLASH_WORD_SIZE,
};
use crate::sync::SpinLock;
use alloc::{string::String, sync::Arc};
use embedded_io::ErrorKind;

pub const ESP32_FLASH_DEVICE_NAME: &str = "esp32-flash0";

// device-specific ioctl commands.
pub const ESP32_FLASH_PREPARE: u32 = 0x40;
pub const ESP32_FLASH_FINALIZE: u32 = 0x41;
pub const ESP32_FLASH_ABORT: u32 = 0x42;
pub const ESP32_FLASH_CLEAR: u32 = 0x43;

// Region base = factory partition end (§13); 4 KiB aligned.
// Size covers all remaining flash: 0x110000 + 0x2F0000 = 0x400000 (4 MB end).
const LOADABLE_IMAGE_OFFSET: u32 = 0x0011_0000;
const LOADABLE_IMAGE_SIZE: u32 = 0x002F_0000;

/// Fixed on-chip flash region; callers use relative offsets.
#[derive(Debug, Clone, Copy)]
pub struct InternalFlashRegion {
    base: u32,
    size: u32,
}

impl InternalFlashRegion {
    pub const fn new(base: u32, size: u32) -> Self {
        Self { base, size }
    }

    pub const fn size(&self) -> u32 {
        self.size
    }

    pub const fn base(&self) -> u32 {
        self.base
    }

    pub fn absolute_offset(&self, relative_offset: u32, len: usize) -> Result<u32, EspFlashError> {
        let relative_end = relative_offset
            .checked_add(len as u32)
            .ok_or(EspFlashError::OutOfBounds)?;
        if relative_end > self.size {
            return Err(EspFlashError::OutOfBounds);
        }
        self.base
            .checked_add(relative_offset)
            .ok_or(EspFlashError::OutOfBounds)
    }

    /// Check alignment + fit.
    pub fn validate(&self, flash_capacity: u32) -> Result<(), EspFlashError> {
        if self.base % ESP_FLASH_SECTOR_SIZE as u32 != 0 {
            return Err(EspFlashError::UnalignedErase);
        }
        if self.size % ESP_FLASH_SECTOR_SIZE as u32 != 0 {
            return Err(EspFlashError::UnalignedErase);
        }
        let end = self
            .base
            .checked_add(self.size)
            .ok_or(EspFlashError::OutOfBounds)?;
        if end > flash_capacity {
            return Err(EspFlashError::OutOfBounds);
        }
        Ok(())
    }
}

fn align_up_sector(value: u32) -> Result<u32, EspFlashError> {
    let sector = ESP_FLASH_SECTOR_SIZE as u32;
    value
        .checked_add(sector - 1)
        .map(|v| v / sector * sector)
        .ok_or(EspFlashError::OutOfBounds)
}

#[derive(Debug)]
enum Esp32FlashImageState {
    Idle,
    Prepared {
        expected_size: u32,
        received_size: u32,
        programmed_size: u32,
        tail: [u8; 4],
        tail_len: usize,
    },
    Ready { image_size: u32 },
}

impl Default for Esp32FlashImageState {
    fn default() -> Self {
        Esp32FlashImageState::Idle
    }
}

/// Misc device wrapping a fixed flash region. State is locked (Device::write is &self).
pub struct Esp32FlashDevice {
    name: String,
    region: InternalFlashRegion,
    state: SpinLock<Esp32FlashImageState>,
}

impl Esp32FlashDevice {
    pub fn new(name: &str, region: InternalFlashRegion) -> Self {
        Self {
            name: String::from(name),
            region,
            state: SpinLock::new(Esp32FlashImageState::Idle),
        }
    }

    /// PREPARE: erase region, reset counters, go Prepared.
    fn ioctl_prepare(&self, image_size: usize) -> Result<(), ErrorKind> {
        if image_size == 0 {
            return Err(ErrorKind::InvalidInput);
        }
        if image_size as u64 > self.region.size() as u64 {
            return Err(ErrorKind::InvalidInput);
        }
        let erase_len = align_up_sector(image_size as u32).map_err(|_| ErrorKind::InvalidInput)?;
        let erase_len = core::cmp::min(erase_len, self.region.size());

        let mut state = self.state.lock();
        if !matches!(*state, Esp32FlashImageState::Idle) {
            return Err(ErrorKind::InvalidInput);
        }
        with_internal_flash(|flash| flash.erase_region(self.region.base(), erase_len))
            .map_err(map_flash_err)?;

        *state = Esp32FlashImageState::Prepared {
            expected_size: image_size as u32,
            received_size: 0,
            programmed_size: 0,
            tail: [0u8; 4],
            tail_len: 0,
        };
        Ok(())
    }

    /// FINALIZE: pad tail with 0xFF, write last word, go Ready.
    fn ioctl_finalize(&self) -> Result<(), ErrorKind> {
        let mut state = self.state.lock();
        let (expected_size, received_size, programmed_size, tail, tail_len) = match &*state {
            Esp32FlashImageState::Prepared {
                expected_size,
                received_size,
                programmed_size,
                tail,
                tail_len,
            } => (*expected_size, *received_size, *programmed_size, *tail, *tail_len),
            _ => return Err(ErrorKind::InvalidInput),
        };
        if received_size != expected_size {
            return Err(ErrorKind::InvalidInput);
        }

        if tail_len > 0 {
            let mut tail = tail;
            for b in tail[tail_len..].iter_mut() {
                *b = 0xFF;
            }
            let phys_off = self
                .region
                .absolute_offset(programmed_size, ESP_FLASH_WORD_SIZE)
                .map_err(|_| ErrorKind::InvalidInput)?;
            with_internal_flash(|flash| flash.program_aligned(phys_off, &tail[..]))
                .map_err(map_flash_err)?;
        }

        *state = Esp32FlashImageState::Ready {
            image_size: expected_size,
        };
        Ok(())
    }

    /// ABORT: drop state, back to Idle.
    fn ioctl_abort(&self) -> Result<(), ErrorKind> {
        let mut state = self.state.lock();
        *state = Esp32FlashImageState::Idle;
        Ok(())
    }

    /// CLEAR: deferred; only resets state for now.
    fn ioctl_clear(&self) -> Result<(), ErrorKind> {
        let mut state = self.state.lock();
        *state = Esp32FlashImageState::Idle;
        Ok(())
    }

    /// Sequential write with 4-byte tail cache (§19).
    fn write_data(&self, pos: u64, buf: &[u8]) -> Result<usize, ErrorKind> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut state = self.state.lock();
        let (expected_size, mut received_size, mut programmed_size, mut tail, mut tail_len) =
            match &*state {
                Esp32FlashImageState::Prepared {
                    expected_size,
                    received_size,
                    programmed_size,
                    tail,
                    tail_len,
                } => (
                    *expected_size,
                    *received_size,
                    *programmed_size,
                    *tail,
                    *tail_len,
                ),
                _ => return Err(ErrorKind::InvalidInput),
            };

        if pos != received_size as u64 {
            return Err(ErrorKind::InvalidInput);
        }
        if (received_size as u64)
            .checked_add(buf.len() as u64)
            .map_or(true, |end| end > expected_size as u64)
        {
            return Err(ErrorKind::InvalidInput);
        }

        let mut consumed = 0usize;
        let data = buf;

        // 1. Fill pending tail to a full word first.
        if tail_len > 0 {
            let need = ESP_FLASH_WORD_SIZE - tail_len;
            let take = core::cmp::min(need, data.len());
            tail[tail_len..tail_len + take].copy_from_slice(&data[..take]);
            tail_len += take;
            consumed += take;
            if tail_len == ESP_FLASH_WORD_SIZE {
                let phys_off = self
                    .region
                    .absolute_offset(programmed_size, ESP_FLASH_WORD_SIZE)
                    .map_err(|_| ErrorKind::InvalidInput)?;
                with_internal_flash(|flash| flash.program_aligned(phys_off, &tail[..]))
                    .map_err(map_flash_err)?;
                programmed_size += ESP_FLASH_WORD_SIZE as u32;
                tail_len = 0;
            }
        }

        // 2. Write word-aligned middle in <=256B chunks.
        let middle_end = consumed + ((data.len() - consumed) & !(ESP_FLASH_WORD_SIZE - 1));
        while consumed < middle_end {
            let n = core::cmp::min(256, middle_end - consumed);
            let phys_off = self
                .region
                .absolute_offset(programmed_size, n)
                .map_err(|_| ErrorKind::InvalidInput)?;
            with_internal_flash(|flash| flash.program_aligned(phys_off, &data[consumed..consumed + n]))
                .map_err(map_flash_err)?;
            programmed_size += n as u32;
            consumed += n;
        }

        // 3. Stash remaining <4 bytes in tail.
        if consumed < data.len() {
            let rem = data.len() - consumed;
            tail[..rem].copy_from_slice(&data[consumed..consumed + rem]);
            tail_len = rem;
            consumed += rem;
        }

        received_size += consumed as u32;

        match &mut *state {
            Esp32FlashImageState::Prepared {
                received_size: rs,
                programmed_size: ps,
                tail: t,
                tail_len: tl,
                ..
            } => {
                *rs = received_size;
                *ps = programmed_size;
                *t = tail;
                *tl = tail_len;
            }
            _ => return Err(ErrorKind::InvalidInput),
        }

        Ok(consumed)
    }
}

impl Device for Esp32FlashDevice {
    fn name(&self) -> String {
        self.name.clone()
    }

    fn class(&self) -> DeviceClass {
        DeviceClass::Misc
    }

    fn id(&self) -> DeviceId {
        DeviceId::new(1, 0x35)
    }

    fn read(&self, pos: u64, buf: &mut [u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        let image_size = match &*self.state.lock() {
            Esp32FlashImageState::Ready { image_size } => *image_size,
            _ => return Err(ErrorKind::Other),
        };
        if pos >= image_size as u64 {
            return Ok(0);
        }
        let avail = (image_size as u64).saturating_sub(pos);
        let n = core::cmp::min(buf.len() as u64, avail) as usize;
        if n == 0 {
            return Ok(0);
        }
        let phys_off = self
            .region
            .absolute_offset(pos as u32, n)
            .map_err(|_| ErrorKind::InvalidInput)?;
        with_internal_flash(|flash| flash.read(phys_off, &mut buf[..n]))
            .map_err(map_flash_err)?;
        Ok(n)
    }

    fn write(&self, pos: u64, buf: &[u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        self.write_data(pos, buf)
    }

    fn ioctl(&self, request: u32, arg: usize) -> Result<(), ErrorKind> {
        match request {
            ESP32_FLASH_PREPARE => self.ioctl_prepare(arg),
            ESP32_FLASH_FINALIZE => self.ioctl_finalize(),
            ESP32_FLASH_ABORT => self.ioctl_abort(),
            ESP32_FLASH_CLEAR => self.ioctl_clear(),
            _ => Err(ErrorKind::Unsupported),
        }
    }

    fn capacity(&self) -> Result<u64, ErrorKind> {
        Ok(self.region.size() as u64)
    }

    fn sector_size(&self) -> Result<u16, ErrorKind> {
        Ok(ESP_FLASH_SECTOR_SIZE as u16)
    }

    fn sync(&self) -> Result<(), ErrorKind> {
        Ok(())
    }
}

fn map_flash_err(e: EspFlashError) -> ErrorKind {
    match e {
        EspFlashError::OutOfBounds | EspFlashError::InvalidLength => ErrorKind::InvalidInput,
        EspFlashError::ProtectedRange => ErrorKind::PermissionDenied,
        EspFlashError::UnalignedErase | EspFlashError::UnalignedWrite => ErrorKind::InvalidInput,
        EspFlashError::Busy => ErrorKind::Other,
        EspFlashError::RomError(_) | EspFlashError::VerifyFailed => ErrorKind::Other,
    }
}

/// Register `/dev/esp32-flash0` (§26).
pub fn init_esp32_flash_device() -> Result<(), ErrorKind> {
    let region = InternalFlashRegion::new(LOADABLE_IMAGE_OFFSET, LOADABLE_IMAGE_SIZE);
    let capacity = with_internal_flash(|flash| Ok(flash.capacity())).map_err(map_flash_err)?;
    region.validate(capacity).map_err(map_flash_err)?;

    let device = Arc::new(Esp32FlashDevice::new(ESP32_FLASH_DEVICE_NAME, region));
    DeviceManager::get()
        .register_device(String::from(ESP32_FLASH_DEVICE_NAME), device)?;
    log::info!(
        "esp32-flash0: region base={:#x} size={:#x}",
        LOADABLE_IMAGE_OFFSET,
        LOADABLE_IMAGE_SIZE
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn region_absolute_offset_basic() {
        let r = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        assert_eq!(r.absolute_offset(0, 0).unwrap(), 0x0011_0000);
        assert_eq!(r.absolute_offset(0x100, 16).unwrap(), 0x0011_0100);
    }

    #[test]
    fn region_absolute_offset_rejects_overflow() {
        let r = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        assert_eq!(
            r.absolute_offset(0x0010_0000, 1),
            Err(EspFlashError::OutOfBounds)
        );
        assert!(r.absolute_offset(0x000F_FFFC, 4).is_ok());
        assert_eq!(
            r.absolute_offset(0x000F_FFFC, 5),
            Err(EspFlashError::OutOfBounds)
        );
    }

    #[test]
    fn region_validate_accepts_aligned_in_range() {
        let r = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        assert!(r.validate(0x0040_0000).is_ok());
    }

    #[test]
    fn region_validate_rejects_unaligned_base() {
        let r = InternalFlashRegion::new(0x0011_0001, 0x0010_0000);
        assert_eq!(r.validate(0x0040_0000), Err(EspFlashError::UnalignedErase));
    }

    #[test]
    fn region_validate_rejects_unaligned_size() {
        let r = InternalFlashRegion::new(0x0011_0000, 0x0010_0001);
        assert_eq!(r.validate(0x0040_0000), Err(EspFlashError::UnalignedErase));
    }

    #[test]
    fn region_validate_rejects_past_capacity() {
        let r = InternalFlashRegion::new(0x0030_0000, 0x0020_0000);
        assert_eq!(r.validate(0x0040_0000), Err(EspFlashError::OutOfBounds));
    }

    #[test]
    fn align_up_sector_basic() {
        assert_eq!(align_up_sector(0).unwrap(), 0);
        assert_eq!(align_up_sector(1).unwrap(), 4096);
        assert_eq!(align_up_sector(4096).unwrap(), 4096);
        assert_eq!(align_up_sector(4097).unwrap(), 8192);
    }

    #[test]
    fn align_up_sector_overflow() {
        assert_eq!(align_up_sector(u32::MAX), Err(EspFlashError::OutOfBounds));
    }

    #[test]
    fn state_default_is_idle() {
        assert!(matches!(
            Esp32FlashImageState::default(),
            Esp32FlashImageState::Idle
        ));
    }

    #[test]
    fn device_construct_keeps_idle() {
        let region = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        let dev = Esp32FlashDevice::new("esp32-flash0", region);
        assert!(matches!(*dev.state.lock(), Esp32FlashImageState::Idle));
    }

    #[test]
    fn device_class_and_names() {
        let region = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        let dev = Esp32FlashDevice::new("esp32-flash0", region);
        assert_eq!(dev.name(), "esp32-flash0");
        assert_eq!(dev.class(), DeviceClass::Misc);
        assert_eq!(dev.capacity().unwrap(), 0x0010_0000);
        assert_eq!(dev.sector_size().unwrap(), ESP_FLASH_SECTOR_SIZE as u16);
    }

    #[test]
    fn read_rejects_not_ready() {
        let region = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        let dev = Esp32FlashDevice::new("esp32-flash0", region);
        let mut buf = [0u8; 8];
        assert_eq!(dev.read(0, &mut buf, false), Err(ErrorKind::Other));
    }

    #[test]
    fn write_rejects_not_prepared() {
        let region = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        let dev = Esp32FlashDevice::new("esp32-flash0", region);
        assert_eq!(dev.write(0, &[0u8; 8], false), Err(ErrorKind::InvalidInput));
    }
}
