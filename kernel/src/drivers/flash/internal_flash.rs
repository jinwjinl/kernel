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

//! ESP32-C3 on-chip main flash raw read/write/erase driver (mask ROM spiflash API).
//! `write()` does NOT auto-erase; callers must `erase_region` first.

use super::esp32_rom;

pub const ESP_FLASH_SECTOR_SIZE: usize = 4096;
pub const ESP_FLASH_WORD_SIZE: usize = 4;
pub const ESP_INTERNAL_FLASH_SIZE: u32 = 4 * 1024 * 1024; // 4 MB
const ROM_PAGE_SIZE: usize = 256; // NOR page-program granularity

/// One-shot init called from boot.rs: unlock the flash once. Per-write re-unlock is
/// not done; a single boot unlock suffices for the raw API.
pub(crate) fn init_internal_flash() -> Result<(), EspFlashError> {
    let r = unsafe { esp32_rom::rom_unlock() };
    if r != esp32_rom::ESP_ROM_SPIFLASH_RESULT_OK {
        log::warn!("esp_rom_spiflash_unlock returned {}", r);
        return Err(EspFlashError::RomError(r));
    }
    let chip_size = unsafe { esp32_rom::rom_chip_size() };
    log::info!(
        "internal flash ROM chip size: {} bytes ({:#x})",
        chip_size,
        chip_size
    );
    Ok(())
}

/// Raw API error type.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum EspFlashError {
    OutOfBounds,
    ProtectedRange,
    InvalidLength,
    UnalignedErase,
    UnalignedWrite,
    Busy,
    RomError(i32), // ROM spiflash non-OK result (1=ERR, 2=TIMEOUT)
    VerifyFailed,
}

fn rom_result(r: i32) -> Result<(), EspFlashError> {
    if r == esp32_rom::ESP_ROM_SPIFLASH_RESULT_OK {
        Ok(())
    } else {
        Err(EspFlashError::RomError(r))
    }
}

pub struct Esp32c3InternalFlash {
    capacity: u32,
}

impl Esp32c3InternalFlash {
    pub const fn new(capacity: u32) -> Self {
        Self { capacity }
    }

    pub const fn capacity(&self) -> u32 {
        self.capacity
    }

    /// `offset + len` must fit within `capacity` (no wraparound).
    fn check_bounds(&self, offset: u32, len: usize) -> Result<(), EspFlashError> {
        let end = offset
            .checked_add(len as u32)
            .ok_or(EspFlashError::OutOfBounds)?;
        if end > self.capacity {
            return Err(EspFlashError::OutOfBounds);
        }
        Ok(())
    }

    /// Read `buf.len()` bytes from `offset`. ROM takes a 4-byte-aligned `*const u32`,
    /// so an align-1 `&mut [u8]` is staged through a 4-aligned word buffer.
    pub fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), EspFlashError> {
        self.check_bounds(offset, buf.len())?;
        if buf.is_empty() {
            return Ok(());
        }
        let aligned = buf.len() & !(ESP_FLASH_WORD_SIZE - 1);
        if aligned != 0 {
            let mut scratch: EspAlignedBuffer<ESP_FLASH_WORD_SIZE> = EspAlignedBuffer::new();
            let mut off = 0usize;
            while off < aligned {
                let r = unsafe {
                    esp32_rom::rom_read(
                        offset + off as u32,
                        scratch.0.as_ptr() as *const u32,
                        ESP_FLASH_WORD_SIZE as u32,
                    )
                };
                rom_result(r)?;
                buf[off..off + ESP_FLASH_WORD_SIZE].copy_from_slice(&scratch.0);
                off += ESP_FLASH_WORD_SIZE;
            }
        }
        // Tail (< 4 bytes): read one aligned word, copy the needed bytes.
        let tail = buf.len() - aligned;
        if tail != 0 {
            let tail_off = offset + aligned as u32;
            let word_off = tail_off & !(ESP_FLASH_WORD_SIZE as u32 - 1);
            let skip = (tail_off - word_off) as usize;
            let mut scratch: EspAlignedBuffer<ESP_FLASH_WORD_SIZE> = EspAlignedBuffer::new();
            let r = unsafe {
                esp32_rom::rom_read(
                    word_off,
                    scratch.0.as_ptr() as *const u32,
                    ESP_FLASH_WORD_SIZE as u32,
                )
            };
            rom_result(r)?;
            buf[aligned..].copy_from_slice(&scratch.0[skip..skip + tail]);
        }
        Ok(())
    }

    /// Erase `len` bytes from `offset`; both must be 4 KB-aligned.
    pub fn erase_region(&mut self, offset: u32, len: u32) -> Result<(), EspFlashError> {
        if offset % ESP_FLASH_SECTOR_SIZE as u32 != 0 {
            return Err(EspFlashError::UnalignedErase);
        }
        if len % ESP_FLASH_SECTOR_SIZE as u32 != 0 {
            return Err(EspFlashError::UnalignedErase);
        }
        self.check_bounds(offset, len as usize)?;
        let first_sector = offset / ESP_FLASH_SECTOR_SIZE as u32;
        let sector_count = len / ESP_FLASH_SECTOR_SIZE as u32;
        for index in 0..sector_count {
            self.erase_sector(first_sector + index)?;
        }
        Ok(())
    }

    fn erase_sector(&mut self, sector: u32) -> Result<(), EspFlashError> {
        // ROM erase_sector takes a sector INDEX (byte_off / 4096), not a byte offset.
        let r = unsafe { esp32_rom::rom_erase_sector(sector) };
        rom_result(r)
    }

    /// Program `data` at `offset`. Does NOT auto-erase (NOR clears bits 1->0 only).
    /// Stages through a 4-aligned page buffer; writes in 256-byte pages defensively.
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), EspFlashError> {
        self.check_bounds(offset, data.len())?;
        if data.is_empty() {
            return Ok(());
        }
        let mut page: EspAlignedBuffer<ROM_PAGE_SIZE> = EspAlignedBuffer::new();
        let mut off = 0usize;
        while off < data.len() {
            let n = core::cmp::min(ROM_PAGE_SIZE, data.len() - off);
            page.0[..n].copy_from_slice(&data[off..off + n]);
            // Pad the final partial page up to a word so ROM gets a 4-aligned len.
            let padded = (n + ESP_FLASH_WORD_SIZE - 1) & !(ESP_FLASH_WORD_SIZE - 1);
            for b in &mut page.0[n..padded] {
                *b = 0xFF; // erased state; padding never clears bits that matter
            }
            let r = unsafe {
                esp32_rom::rom_write(
                    offset + off as u32,
                    page.0.as_ptr() as *const u32,
                    padded as u32,
                )
            };
            rom_result(r)?;
            off += n;
        }
        Ok(())
    }
}

/// 4-byte-aligned scratch buffer, generic on `N` for word/page reuse.
#[repr(align(4))]
struct EspAlignedBuffer<const N: usize>([u8; N]);

impl<const N: usize> EspAlignedBuffer<N> {
    const fn new() -> Self {
        Self([0u8; N])
    }
}

#[cfg(test)]
mod tests {
    use super::{super::esp32_rom, *};
    use blueos_test_macro::test;

    #[test]
    fn new_sets_capacity() {
        let f = Esp32c3InternalFlash::new(0x0040_0000);
        assert_eq!(f.capacity(), 0x0040_0000);
    }

    #[test]
    fn check_bounds_accepts_in_range() {
        let f = Esp32c3InternalFlash::new(4096);
        assert!(f.check_bounds(0, 4096).is_ok());
        assert!(f.check_bounds(0, 0).is_ok());
        assert!(f.check_bounds(4096, 0).is_ok()); // zero-len at the edge
    }

    #[test]
    fn check_bounds_rejects_overflow() {
        let f = Esp32c3InternalFlash::new(4096);
        assert_eq!(f.check_bounds(0, 4097), Err(EspFlashError::OutOfBounds));
        assert_eq!(f.check_bounds(1, 4096), Err(EspFlashError::OutOfBounds));
        // offset+len must not wrap around to Ok
        assert_eq!(f.check_bounds(u32::MAX, 1), Err(EspFlashError::OutOfBounds));
    }

    #[test]
    fn erase_region_requires_sector_alignment() {
        let mut f = Esp32c3InternalFlash::new(0x0040_0000);
        assert_eq!(f.erase_region(1, 4096), Err(EspFlashError::UnalignedErase));
        assert_eq!(f.erase_region(0, 1), Err(EspFlashError::UnalignedErase));
        // alignment is checked first, so an unaligned offset is rejected even for len 0
        assert_eq!(f.erase_region(1, 0), Err(EspFlashError::UnalignedErase));
    }

    #[test]
    fn erase_region_rejects_out_of_bounds() {
        let mut f = Esp32c3InternalFlash::new(0x0040_0000);
        // aligned but past capacity
        assert_eq!(
            f.erase_region(0x0040_0000, 4096),
            Err(EspFlashError::OutOfBounds)
        );
    }

    #[test]
    fn rom_result_maps_codes() {
        assert_eq!(rom_result(esp32_rom::ESP_ROM_SPIFLASH_RESULT_OK), Ok(()));
        assert_eq!(
            rom_result(esp32_rom::ESP_ROM_SPIFLASH_RESULT_ERR),
            Err(EspFlashError::RomError(1))
        );
        assert_eq!(
            rom_result(esp32_rom::ESP_ROM_SPIFLASH_RESULT_TIMEOUT),
            Err(EspFlashError::RomError(2))
        );
    }

    #[test]
    fn aligned_buffer_is_word_aligned() {
        let buf: EspAlignedBuffer<128> = EspAlignedBuffer::new();
        let addr = buf.0.as_ptr() as usize;
        assert_eq!(addr % ESP_FLASH_WORD_SIZE, 0);
    }
}
