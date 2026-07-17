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

//! On-chip mask ROM spiflash + cache FFI and IRAM-resident guard wrappers.
//! Each ROM call is wrapped in the unicore flash-op guard: disable IRQ -> suspend
//! ICache -> ROM call -> resume ICache -> restore IRQ. The `.rwtext` wrappers must
//! live in IRAM: while flash is busy (erase/program) the CPU cannot fetch from flash.

use crate::arch::{disable_local_irq_save, enable_local_irq_restore};

pub const ESP_ROM_SPIFLASH_RESULT_OK: i32 = 0;
pub const ESP_ROM_SPIFLASH_RESULT_ERR: i32 = 1;
pub const ESP_ROM_SPIFLASH_RESULT_TIMEOUT: i32 = 2;

// PROVIDE'd by esp32c3.rom.ld, reached via libesp_rom_sys.a -> rom-functions.x.
unsafe extern "C" {
    pub fn esp_rom_spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32;
    pub fn esp_rom_spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32;
    pub fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32; // INDEX (byte_off / 4096)
    pub fn esp_rom_spiflash_erase_block(block_number: u32) -> i32; // 64KB INDEX
    pub fn esp_rom_spiflash_unlock() -> i32;
    fn spi_flash_get_chip_size() -> u32; // ROM-detected flash size in bytes (bootloader-filled)
    fn Cache_Suspend_ICache() -> u32; // returns autoload state
    fn Cache_Resume_ICache(state: u32);
    fn Cache_Invalidate_Addr(vaddr: u32, len: u32);
    fn Cache_Invalidate_ICache_All();
}

const DROM_VADDR_BASE: u32 = 0x3C00_0000;

/// Run `body` with IRQs disabled and ICache suspended across the ROM call.
#[inline(always)]
fn with_flash_op<R>(body: impl FnOnce() -> R) -> R {
    let flags = disable_local_irq_save();
    let cache_state = unsafe { Cache_Suspend_ICache() };
    let result = body();
    unsafe { Cache_Resume_ICache(cache_state) };
    enable_local_irq_restore(flags);
    result
}

#[link_section = ".rwtext"]
#[inline(never)]
pub(crate) unsafe fn rom_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    with_flash_op(|| unsafe { esp_rom_spiflash_read(src_addr, data, len) })
}

#[link_section = ".rwtext"]
#[inline(never)]
pub(crate) unsafe fn rom_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    let r = with_flash_op(|| unsafe { esp_rom_spiflash_write(dest_addr, data, len) });
    if r == ESP_ROM_SPIFLASH_RESULT_OK {
        // Drop a stale I-cache line backing the written region in case it is ever executed.
        let vaddr = DROM_VADDR_BASE.wrapping_add(dest_addr);
        unsafe { Cache_Invalidate_Addr(vaddr, len) };
    }
    r
}

#[link_section = ".rwtext"]
#[inline(never)]
pub(crate) unsafe fn rom_erase_sector(sector_index: u32) -> i32 {
    with_flash_op(|| unsafe { esp_rom_spiflash_erase_sector(sector_index) })
}

#[link_section = ".rwtext"]
#[inline(never)]
pub(crate) unsafe fn rom_erase_block(block_index: u32) -> i32 {
    with_flash_op(|| unsafe { esp_rom_spiflash_erase_block(block_index) })
}

// Called once at init with interrupts live; not cache-protected (one-shot reg clear).
pub(crate) unsafe fn rom_unlock() -> i32 {
    unsafe { esp_rom_spiflash_unlock() }
}

// ROM-detected flash capacity (filled by 1st-stage bootloader). Read-only query,
// no erase/program, so no cache guard needed.
pub(crate) unsafe fn rom_chip_size() -> u32 {
    unsafe { spi_flash_get_chip_size() }
}
