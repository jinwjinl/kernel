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
use crate::sync::SpinLock;

pub const ESP_FLASH_SECTOR_SIZE: usize = 4096;
pub const ESP_FLASH_WORD_SIZE: usize = 4;
const ROM_PAGE_SIZE: usize = 256; // NOR page-program granularity
const ESP_FLASH_READ_CHUNK_SIZE: usize = 1024; // batch read granularity

/// One-shot boot unlock. Per-write re-unlock is unnecessary for the raw API.
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

/// Cross-thread serialization for flash ops; ROM wrappers only guard one call.
static INTERNAL_FLASH_LOCK: SpinLock<()> = SpinLock::new(());

pub fn with_internal_flash<R>(
    operation: impl FnOnce(&mut Esp32c3InternalFlash) -> Result<R, EspFlashError>,
) -> Result<R, EspFlashError> {
    let _guard = INTERNAL_FLASH_LOCK.lock();
    let mut flash = Esp32c3InternalFlash::detect()?;
    operation(&mut flash)
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

    /// Construct on demand from the ROM chip size; unlock stays in `init_internal_flash`.
    pub fn detect() -> Result<Self, EspFlashError> {
        let size = unsafe { esp32_rom::rom_chip_size() };
        Ok(Self::new(size))
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

    /// Read `buf.len()` bytes from `offset`, batched in 1 KiB chunks. ROM takes
    /// a 4-aligned `*const u32`, so unaligned head/tail are staged through a word buffer.
    pub fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), EspFlashError> {
        self.check_bounds(offset, buf.len())?;
        if buf.is_empty() {
            return Ok(());
        }

        // Head: align `offset` down to a word, copy the needed leading bytes.
        let mut done = 0usize;
        let head_mis = (offset as usize) & (ESP_FLASH_WORD_SIZE - 1);
        if head_mis != 0 {
            let word_off = offset & !(ESP_FLASH_WORD_SIZE as u32 - 1);
            let head_len = core::cmp::min(ESP_FLASH_WORD_SIZE - head_mis, buf.len());
            let mut scratch: EspAlignedBuffer<ESP_FLASH_WORD_SIZE> = EspAlignedBuffer::new();
            let r = unsafe {
                esp32_rom::rom_read(
                    word_off,
                    scratch.0.as_ptr() as *const u32,
                    ESP_FLASH_WORD_SIZE as u32,
                )
            };
            rom_result(r)?;
            buf[..head_len].copy_from_slice(&scratch.0[head_mis..head_mis + head_len]);
            done = head_len;
        }

        // Middle: 1 KiB chunks, 4-aligned length.
        let mut chunk: EspAlignedBuffer<ESP_FLASH_READ_CHUNK_SIZE> = EspAlignedBuffer::new();
        while done < buf.len() {
            let remaining = buf.len() - done;
            if remaining < ESP_FLASH_WORD_SIZE {
                break; // <4B tail handled below
            }
            let n = core::cmp::min(ESP_FLASH_READ_CHUNK_SIZE, remaining & !(ESP_FLASH_WORD_SIZE - 1));
            let r = unsafe {
                esp32_rom::rom_read(
                    offset + done as u32,
                    chunk.0.as_ptr() as *const u32,
                    n as u32,
                )
            };
            rom_result(r)?;
            buf[done..done + n].copy_from_slice(&chunk.0[..n]);
            done += n;
        }

        // Tail (< 4 bytes): read one aligned word, copy the needed bytes.
        if done < buf.len() {
            let tail_off = offset + done as u32;
            let word_off = tail_off & !(ESP_FLASH_WORD_SIZE as u32 - 1);
            let skip = (tail_off - word_off) as usize;
            let tail = buf.len() - done;
            let mut scratch: EspAlignedBuffer<ESP_FLASH_WORD_SIZE> = EspAlignedBuffer::new();
            let r = unsafe {
                esp32_rom::rom_read(
                    word_off,
                    scratch.0.as_ptr() as *const u32,
                    ESP_FLASH_WORD_SIZE as u32,
                )
            };
            rom_result(r)?;
            buf[done..].copy_from_slice(&scratch.0[skip..skip + tail]);
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

    /// Strict program: `offset` and `data.len()` must be 4-aligned; the
    /// region must be pre-erased (NOR 1->0). No auto-erase, no tail padding. Writes
    /// are split on 256-byte page boundaries so no ROM call crosses a page.
    pub fn program_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), EspFlashError> {
        if offset % ESP_FLASH_WORD_SIZE as u32 != 0 {
            return Err(EspFlashError::UnalignedWrite);
        }
        if data.len() % ESP_FLASH_WORD_SIZE != 0 {
            return Err(EspFlashError::UnalignedWrite);
        }
        self.check_bounds(offset, data.len())?;
        if data.is_empty() {
            return Ok(());
        }
        let mut page: EspAlignedBuffer<ROM_PAGE_SIZE> = EspAlignedBuffer::new();
        let mut done = 0usize;
        while done < data.len() {
            let current_offset = offset + done as u32;
            let offset_in_page = (current_offset as usize) % ROM_PAGE_SIZE;
            let page_remaining = ROM_PAGE_SIZE - offset_in_page;
            let write_len = core::cmp::min(page_remaining, data.len() - done);
            // Staged through a 4-aligned buffer: ROM takes *const u32.
            page.0[..write_len].copy_from_slice(&data[done..done + write_len]);
            let r = unsafe {
                esp32_rom::rom_write(
                    current_offset,
                    page.0.as_ptr() as *const u32,
                    write_len as u32,
                )
            };
            rom_result(r)?;
            done += write_len;
        }
        Ok(())
    }

    /// Thin passthrough to `program_aligned` (retained legacy name, no padding).
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), EspFlashError> {
        self.program_aligned(offset, data)
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

    #[test]
    fn program_aligned_rejects_unaligned_offset() {
        let mut f = Esp32c3InternalFlash::new(0x0040_0000);
        let data = [0u8; 8]; // 4-aligned length
        assert_eq!(f.program_aligned(1, &data), Err(EspFlashError::UnalignedWrite));
        assert_eq!(f.program_aligned(2, &data), Err(EspFlashError::UnalignedWrite));
        assert_eq!(f.program_aligned(3, &data), Err(EspFlashError::UnalignedWrite));
    }

    #[test]
    fn program_aligned_rejects_unaligned_len() {
        let mut f = Esp32c3InternalFlash::new(0x0040_0000);
        assert_eq!(f.program_aligned(0, &[0u8; 3]), Err(EspFlashError::UnalignedWrite));
        assert_eq!(f.program_aligned(0, &[0u8; 5]), Err(EspFlashError::UnalignedWrite));
        assert_eq!(f.program_aligned(0, &[0u8; 7]), Err(EspFlashError::UnalignedWrite));
    }

    #[test]
    fn program_aligned_rejects_out_of_bounds() {
        let mut f = Esp32c3InternalFlash::new(0x0040_0000); // 4 MB
        // Aligned but past capacity: offset+len must not exceed capacity.
        assert_eq!(
            f.program_aligned(0x0040_0000, &[0u8; 8]),
            Err(EspFlashError::OutOfBounds)
        );
    }

    #[test]
    fn program_aligned_accepts_aligned_empty() {
        let mut f = Esp32c3InternalFlash::new(0x0040_0000);
        // Empty data short-circuits before any ROM call.
        assert_eq!(f.program_aligned(0, &[]), Ok(()));
        assert_eq!(f.program_aligned(4, &[]), Ok(()));
    }
}
