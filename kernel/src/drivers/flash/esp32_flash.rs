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

use crate::{
    devices::{Device, DeviceClass, DeviceId, DeviceManager},
    drivers::flash::{
        esp32_rom, flash_mmap::{self, ExecMapping, MapError},
        internal_flash::{
            with_internal_flash, EspFlashError, ESP_FLASH_SECTOR_SIZE, ESP_FLASH_WORD_SIZE,
        },
    },
    sync::SpinLock,
};
use alloc::{string::String, sync::Arc};
use embedded_io::ErrorKind;

pub const ESP32_FLASH_DEVICE_NAME: &str = "esp32-flash0";

// device-specific ioctl commands.
pub const ESP32_FLASH_PREPARE: u32 = 0x40;
pub const ESP32_FLASH_FINALIZE: u32 = 0x41;
pub const ESP32_FLASH_ABORT: u32 = 0x42;
pub const ESP32_FLASH_CLEAR: u32 = 0x43;
pub const ESP32_FLASH_MAP: u32 = 0x44;
pub const ESP32_FLASH_UNMAP: u32 = 0x45;
// Local/experimental: not merged to mainline. Returns the kernel DRAM safe base
// (__sys_stack_end) so the userspace loader knows where free SRAM begins for
// copying out-of-window RW segments at load time.
pub const ESP32_FLASH_QUERY_DRAM_SAFE: u32 = 0x46;
// Local/diagnostic: reads 4 bytes back through the arg out-pointer to localize
// the 0x4220185E illegal-instruction trap. Read-only, no device state change.
pub const ESP32_FLASH_PROBE_LMA: u32 = 0x47;

// Loadable Region: factory partition end .. 4 MB flash end.
// Single source of truth; partition table `factory` end MUST equal LOADABLE_REGION_BASE.
pub const LOADABLE_REGION_BASE: u32 = 0x0011_0000;
pub const LOADABLE_REGION_SIZE: u32 = 0x002F_0000;
pub const LOADABLE_REGION_END: u32 = LOADABLE_REGION_BASE + LOADABLE_REGION_SIZE;

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
    Ready {
        image_size: u32,
    },
    Mapped {
        mapping: ExecMapping,
        image_size: u32,
    },
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
            // Mapped refuses reinstall; other non-Idle is a usage error.
            let err = match &*state {
                Esp32FlashImageState::Mapped { .. } => ErrorKind::PermissionDenied,
                _ => ErrorKind::InvalidInput,
            };
            return Err(err);
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
            } => (
                *expected_size,
                *received_size,
                *programmed_size,
                *tail,
                *tail_len,
            ),
            Esp32FlashImageState::Mapped { .. } => return Err(ErrorKind::PermissionDenied),
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

    /// ABORT: drop state, back to Idle. Refused while Mapped (unmap first).
    fn ioctl_abort(&self) -> Result<(), ErrorKind> {
        let mut state = self.state.lock();
        if matches!(*state, Esp32FlashImageState::Mapped { .. }) {
            return Err(ErrorKind::PermissionDenied);
        }
        *state = Esp32FlashImageState::Idle;
        Ok(())
    }

    /// CLEAR: deferred; only resets state for now. Refused while Mapped (unmap first).
    fn ioctl_clear(&self) -> Result<(), ErrorKind> {
        let mut state = self.state.lock();
        if matches!(*state, Esp32FlashImageState::Mapped { .. }) {
            return Err(ErrorKind::PermissionDenied);
        }
        *state = Esp32FlashImageState::Idle;
        Ok(())
    }

    /// Sequential write with 4-byte tail cache.
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
                Esp32FlashImageState::Mapped { .. } => return Err(ErrorKind::PermissionDenied),
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
            with_internal_flash(|flash| {
                flash.program_aligned(phys_off, &data[consumed..consumed + n])
            })
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

    /// MAP: map the whole Ready image as executable, go Mapped. When `arg` is a
    /// non-null `*mut u32` from userspace, write the segment address back so the
    /// caller can transmute it into an entry pointer. arg==0 means "don't care".
    fn ioctl_map(&self, arg: usize) -> Result<(), ErrorKind> {
        let mut state = self.state.lock();
        let image_size = match &*state {
            Esp32FlashImageState::Ready { image_size } => *image_size,
            Esp32FlashImageState::Mapped { .. } => return Err(ErrorKind::PermissionDenied),
            _ => return Err(ErrorKind::InvalidInput),
        };
        // Fixed mapping: entire image at region base (single live mapping).
        let mapping =
            flash_mmap::map_exec(self.region.base(), image_size as usize).map_err(map_mmap_err)?;
        if arg != 0 {
            // SAFETY: caller (userspace) passes a valid writable u32. No kernel
            // copy_to_user helper exists; the tmpfs ioctl bridge passes arg
            // through unchecked, matching existing ioctl conventions.
            unsafe { *(arg as *mut u32) = mapping.segment_address as u32 };
        }
        *state = Esp32FlashImageState::Mapped {
            mapping,
            image_size,
        };
        Ok(())
    }

    /// UNMAP: release mapping, back to Ready. Caller MUST not execute in region.
    fn ioctl_unmap(&self) -> Result<(), ErrorKind> {
        let mut state = self.state.lock();
        let old = core::mem::replace(&mut *state, Esp32FlashImageState::Idle);
        let (mapping, image_size) = match old {
            Esp32FlashImageState::Mapped {
                mapping,
                image_size,
            } => (mapping, image_size),
            other => {
                *state = other;
                return Err(ErrorKind::InvalidInput);
            }
        };
        // Set Ready before calling flash_mmap to avoid nesting state lock inside MMAP_STATE.
        *state = Esp32FlashImageState::Ready { image_size };
        drop(state);
        match flash_mmap::unmap_exec(&mapping).map_err(map_mmap_err) {
            Ok(()) => Ok(()),
            Err(e) => {
                // Roll back: keep mapping live so caller can retry.
                let mut state = self.state.lock();
                *state = Esp32FlashImageState::Mapped {
                    mapping,
                    image_size,
                };
                Err(e)
            }
        }
    }

    /// QUERY_DRAM_SAFE: write the kernel DRAM safe base (__sys_stack_end) to the
    /// caller's *mut u32 so the loader knows where free SRAM for RW segments
    /// begins. Local/experimental ioctl, not merged to mainline. Read-only symbol
    /// query, independent of device state, so no state lock needed.
    fn ioctl_query_dram_safe(&self, arg: usize) -> Result<(), ErrorKind> {
        if arg != 0 {
            // SAFETY: caller passes a valid writable u32 (tmpfs ioctl bridge passes
            // arg through unchecked, matching existing ioctl_map conventions).
            // addr_of! avoids borrowing a `static mut` (static_mut_refs lint);
            // boot.rs uses the same idiom for these linker symbols.
            let safe = core::ptr::addr_of!(crate::boot::__sys_stack_end) as u32;
            unsafe { *(arg as *mut u32) = safe };
        }
        Ok(())
    }

    /// PROBE_LMA: diagnostic. Reads 4 values at the known trap point and writes
    /// them back through the arg out-pointer so the loader can print them. Read-
    /// only, independent of device state, so no state lock. Local/diagnostic,
    /// not merged to mainline.
    fn ioctl_probe_lma(&self, arg: usize) -> Result<(), ErrorKind> {
        // Fixed trap point: vaddr 0x4220185E, MMU entry 32, physical 0x20185E,
        // D-bus view 0x3C20185E. All derived from the observed illegal-instr trap.
        const TRAP_VADDR: u32 = 0x4220_185E;
        const TRAP_ENTRY: u32 = 32;
        const TRAP_PHYS: u32 = 0x0020_185E;
        const TRAP_DBUS: u32 = 0x3C20_185E;

        let entry32 = unsafe { esp32_rom::rom_mmu_entry_read(TRAP_ENTRY) };

        // Cache-bypass physical read of one byte. rom_read wants a word-aligned
        // buffer; read 4 bytes and take the first.
        let mut phys_word: u32 = 0xFFFF_FFFF;
        let _ = unsafe {
            esp32_rom::rom_read(
                TRAP_PHYS & !0x3,
                &mut phys_word as *mut u32 as *const u32,
                4,
            )
        };
        let phys_byte = (phys_word >> ((TRAP_PHYS & 0x3) * 8)) as u8 as u32;

        // D-bus and I-bus vaddr reads go through the Flash MMU + ICache (the same
        // path fetch uses), so a 0xFF here is the real trap signal.
        // SAFETY: map_exec has run and invalidated ICache; the vaddr is mapped.
        // A load does not trap on illegal instruction data; an unmapped entry
        // would raise access-fault (also useful diagnostic), so this never hangs.
        let dbus_byte = unsafe { core::ptr::read_volatile(TRAP_DBUS as *const u8) as u32 };
        let ibus_byte = unsafe { core::ptr::read_volatile(TRAP_VADDR as *const u8) as u32 };

        if arg != 0 {
            // SAFETY: caller passes a valid writable 16-byte ProbeResult; the VFS
            // ioctl bridge passes arg through unchecked (same as ioctl_map).
            let out: [u32; 4] = [entry32, phys_byte, dbus_byte, ibus_byte];
            unsafe {
                core::ptr::copy_nonoverlapping(
                    out.as_ptr(),
                    arg as *mut u32,
                    out.len(),
                );
            }
        }
        Ok(())
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
            Esp32FlashImageState::Mapped { image_size, .. } => *image_size,
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
        with_internal_flash(|flash| flash.read(phys_off, &mut buf[..n])).map_err(map_flash_err)?;
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
            ESP32_FLASH_MAP => self.ioctl_map(arg),
            ESP32_FLASH_UNMAP => self.ioctl_unmap(),
            ESP32_FLASH_QUERY_DRAM_SAFE => self.ioctl_query_dram_safe(arg),
            ESP32_FLASH_PROBE_LMA => self.ioctl_probe_lma(arg),
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

fn map_mmap_err(e: MapError) -> ErrorKind {
    match e {
        MapError::AlreadyMapped => ErrorKind::PermissionDenied,
        MapError::ZeroSize
        | MapError::OutOfRange
        | MapError::Overflow
        | MapError::InvalidHandle => ErrorKind::InvalidInput,
    }
}

pub fn init_esp32_flash_device() -> Result<(), ErrorKind> {
    let region = InternalFlashRegion::new(LOADABLE_REGION_BASE, LOADABLE_REGION_SIZE);
    let capacity = with_internal_flash(|flash| Ok(flash.capacity())).map_err(map_flash_err)?;
    region.validate(capacity).map_err(map_flash_err)?;

    let device = Arc::new(Esp32FlashDevice::new(ESP32_FLASH_DEVICE_NAME, region));
    DeviceManager::get().register_device(String::from(ESP32_FLASH_DEVICE_NAME), device)?;
    log::info!(
        "esp32-flash0: region base={:#x} size={:#x}",
        LOADABLE_REGION_BASE,
        LOADABLE_REGION_SIZE
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

    // Helper: build a device whose state is forced to Mapped, bypassing the
    // ROM/FFI path of ioctl_map (unavailable on host tests).
    fn mapped_device(image_size: u32) -> Esp32FlashDevice {
        let region = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        let dev = Esp32FlashDevice::new("esp32-flash0", region);
        *dev.state.lock() = Esp32FlashImageState::Mapped {
            mapping: ExecMapping::for_test(),
            image_size,
        };
        dev
    }

    #[test]
    fn mapped_refuses_prepare() {
        let dev = mapped_device(0x1000);
        assert_eq!(dev.ioctl_prepare(0x1000), Err(ErrorKind::PermissionDenied));
    }

    #[test]
    fn mapped_refuses_write() {
        let dev = mapped_device(0x1000);
        assert_eq!(
            dev.write(0, &[0u8; 8], false),
            Err(ErrorKind::PermissionDenied)
        );
    }

    #[test]
    fn mapped_refuses_finalize() {
        let dev = mapped_device(0x1000);
        assert_eq!(dev.ioctl_finalize(), Err(ErrorKind::PermissionDenied));
    }

    #[test]
    fn mapped_refuses_abort() {
        let dev = mapped_device(0x1000);
        assert_eq!(dev.ioctl_abort(), Err(ErrorKind::PermissionDenied));
    }

    #[test]
    fn mapped_refuses_clear() {
        let dev = mapped_device(0x1000);
        assert_eq!(dev.ioctl_clear(), Err(ErrorKind::PermissionDenied));
    }

    #[test]
    fn mapped_refuses_map() {
        // Double-map at device layer must refuse even before hitting flash_mmap.
        let dev = mapped_device(0x1000);
        assert_eq!(dev.ioctl_map(0), Err(ErrorKind::PermissionDenied));
    }

    #[test]
    fn map_rejects_not_ready() {
        let region = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        let dev = Esp32FlashDevice::new("esp32-flash0", region);
        // Idle: not Ready. Avoids the ROM path (state check happens first).
        assert_eq!(dev.ioctl_map(0), Err(ErrorKind::InvalidInput));
    }

    #[test]
    fn unmap_rejects_not_mapped() {
        let region = InternalFlashRegion::new(0x0011_0000, 0x0010_0000);
        let dev = Esp32FlashDevice::new("esp32-flash0", region);
        assert_eq!(dev.ioctl_unmap(), Err(ErrorKind::InvalidInput));
    }

    #[test]
    fn read_accepts_mapped() {
        // Mapped state should allow read (XIP does not block reads). Probe the
        // in-range/edge path without touching the real flash backend: pos ==
        // image_size yields Ok(0) before any hardware call.
        let dev = mapped_device(0x1000);
        let mut buf = [0u8; 8];
        assert_eq!(dev.read(0x1000, &mut buf, false), Ok(0));
    }

    #[test]
    fn map_mmap_err_mapping() {
        assert_eq!(
            map_mmap_err(MapError::AlreadyMapped),
            ErrorKind::PermissionDenied
        );
        assert_eq!(map_mmap_err(MapError::ZeroSize), ErrorKind::InvalidInput);
        assert_eq!(map_mmap_err(MapError::OutOfRange), ErrorKind::InvalidInput);
        assert_eq!(map_mmap_err(MapError::Overflow), ErrorKind::InvalidInput);
        assert_eq!(
            map_mmap_err(MapError::InvalidHandle),
            ErrorKind::InvalidInput
        );
    }
}
