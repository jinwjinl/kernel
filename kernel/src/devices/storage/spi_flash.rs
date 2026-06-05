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

//! SPI NOR Flash FTL (Flash Translation Layer) block driver adapter
//!
//! Adapts the SPI NOR Flash command layer (`SpiFlashCmd`) to BlueKernel's
//! `BlockDriverOps` trait, hiding erase-before-write semantics with a
//! single erase-block cache strategy.

use alloc::{string::String, sync::Arc, vec, vec::Vec};
use core::cmp::min;
use embedded_hal::spi::SpiDevice;
use embedded_io::ErrorKind;

use crate::{
    devices::{
        block::{Block, BlockDriverOps, BlockError, ErrorType},
        storage::spi_flash_cmd::{FlashError, SpiFlashCmd},
        DeviceManager,
    },
    sync::SpinLock,
};

/// Flash block device sector size (512 bytes)
const FLASH_SECTOR_SIZE: u16 = 512;

/// Flash block device name registered with DeviceManager
const FLASH_STORAGE_NAME: &str = "flash-storage";

/// Erase block size (4KB) — matches sector_erase (0x20) granularity
const FLASH_ERASE_SIZE: usize = 4096;

/// Number of pages per erase block (4096 / 256 = 16)
const PAGES_PER_ERASE_BLOCK: usize = FLASH_ERASE_SIZE / 256;

/// Flash block driver error
#[derive(Debug, Clone, Eq, PartialEq, thiserror::Error)]
pub enum FlashBlockError {
    /// Error originating from the SPI Flash command layer
    #[error("Flash error: {0}")]
    Flash(#[from] FlashError),
}

/// SPI NOR Flash FTL block driver
///
/// Implements BlueKernel's `BlockDriverOps` trait by caching one erase block
/// (4KB) at a time. Writes are buffered in the cache and only committed to
/// Flash on `flush()` or when a different erase block is accessed.
///
/// This hides the erase-before-write semantics of NOR Flash from upper layers.
pub struct SpiFlashBlockDriver<SPI: SpiDevice<u8>> {
    flash_cmd: SpiFlashCmd<SPI>,
    capacity_bytes: u64,
    erase_size: usize,
    erase_buf: Vec<u8>,
    dirty: bool,
    current_erase_block: Option<usize>,
}

impl<SPI: SpiDevice<u8> + Send> SpiFlashBlockDriver<SPI> {
    /// Create a new SPI Flash block driver
    ///
    /// `capacity_bytes` should reflect the actual Flash capacity (derived from
    /// the JEDEC ID density byte).
    pub fn new(flash_cmd: SpiFlashCmd<SPI>, capacity_bytes: u64) -> Self {
        SpiFlashBlockDriver {
            flash_cmd,
            capacity_bytes,
            erase_size: FLASH_ERASE_SIZE,
            erase_buf: vec![0u8; FLASH_ERASE_SIZE],
            dirty: false,
            current_erase_block: None,
        }
    }

    /// Read an entire erase block (4KB) from Flash into the cache
    fn read_erase_block(&mut self, erase_block_id: usize) -> Result<(), FlashError> {
        let addr = erase_block_id * FLASH_ERASE_SIZE;
        self.flash_cmd.read(addr as u32, &mut self.erase_buf)?;
        self.current_erase_block = Some(erase_block_id);
        self.dirty = false;
        Ok(())
    }

    /// Flush the current erase block cache to Flash if dirty
    ///
    /// Performs: sector_erase → page_program (16 pages of 256 bytes)
    fn flush_erase_block(&mut self) -> Result<(), FlashError> {
        if !self.dirty || self.current_erase_block.is_none() {
            return Ok(());
        }
        let erase_block_id = self.current_erase_block.unwrap();
        let addr = (erase_block_id * FLASH_ERASE_SIZE) as u32;

        // Erase the 4KB sector
        self.flash_cmd.sector_erase(addr)?;

        // Program all 16 pages (4096 / 256 = 16)
        for page_idx in 0..PAGES_PER_ERASE_BLOCK {
            let page_offset = page_idx * 256;
            let page_data = &self.erase_buf[page_offset..page_offset + 256];
            self.flash_cmd
                .page_program(addr + page_offset as u32, page_data)?;
        }

        self.dirty = false;
        Ok(())
    }

    /// Ensure the cache holds the erase block containing the given block_id
    ///
    /// If the target erase block differs from the cached one, flush the
    /// current cache first and then read the new erase block.
    fn ensure_erase_block(&mut self, block_id: usize) -> Result<(), FlashError> {
        let erase_block_id = block_id / (FLASH_ERASE_SIZE / FLASH_SECTOR_SIZE as usize);
        if self.current_erase_block != Some(erase_block_id) {
            self.flush_erase_block()?;
            self.read_erase_block(erase_block_id)?;
        }
        Ok(())
    }

    /// Compute the offset within the erase_buf for a given block_id
    fn block_offset_in_erase(&self, block_id: usize) -> usize {
        (block_id % (FLASH_ERASE_SIZE / FLASH_SECTOR_SIZE as usize)) * FLASH_SECTOR_SIZE as usize
    }
}

impl<SPI: SpiDevice<u8> + Send + Sync> ErrorType for SpiFlashBlockDriver<SPI> {
    type Error = BlockError<FlashBlockError>;
}

impl<SPI: SpiDevice<u8> + Send + Sync> BlockDriverOps for SpiFlashBlockDriver<SPI> {
    fn capacity(&self) -> u64 {
        self.capacity_bytes / FLASH_SECTOR_SIZE as u64
    }

    fn sector_size(&self) -> u16 {
        FLASH_SECTOR_SIZE
    }

    fn read_blocks(&mut self, block_id: usize, buf: &mut [u8]) -> Result<(), Self::Error> {
        let erase_block_id = block_id / (FLASH_ERASE_SIZE / FLASH_SECTOR_SIZE as usize);

        // If dirty cache overlaps the requested block, read from cache
        if self.dirty && self.current_erase_block == Some(erase_block_id) {
            let offset = self.block_offset_in_erase(block_id);
            let copy_len = min(buf.len(), FLASH_ERASE_SIZE - offset);
            buf[..copy_len].copy_from_slice(&self.erase_buf[offset..offset + copy_len]);
            return Ok(());
        }

        // Otherwise read directly from Flash
        let addr = (block_id * FLASH_SECTOR_SIZE as usize) as u32;
        self.flash_cmd
            .read(addr, buf)
            .map_err(|e| BlockError::Driver(FlashBlockError::Flash(e)))?;
        Ok(())
    }

    fn write_blocks(&mut self, block_id: usize, buf: &[u8]) -> Result<(), Self::Error> {
        // Ensure the correct erase block is cached
        self.ensure_erase_block(block_id)
            .map_err(|e| BlockError::Driver(FlashBlockError::Flash(e)))?;

        // Update the cache with new data
        let offset = self.block_offset_in_erase(block_id);
        let copy_len = min(buf.len(), FLASH_ERASE_SIZE - offset);
        self.erase_buf[offset..offset + copy_len].copy_from_slice(&buf[..copy_len]);
        self.dirty = true;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_erase_block()
            .map_err(|e| BlockError::Driver(FlashBlockError::Flash(e)))?;
        Ok(())
    }
}

/// Initialize the SPI NOR Flash block device
///
/// Reads the JEDEC ID, determines capacity from the density byte,
/// creates the FTL block driver, and registers it with DeviceManager.
pub fn init_spi_flash<SPI>(spi: SPI) -> Result<(), ErrorKind>
where
    SPI: SpiDevice<u8> + Send + Sync + 'static,
{
    let mut flash_cmd = SpiFlashCmd::new(spi);

    // Read JEDEC ID to determine capacity
    let jedec_id = flash_cmd.jedec_id().map_err(|e| match e {
        FlashError::Spi(_) => ErrorKind::Other,
        FlashError::Timeout => ErrorKind::TimedOut,
        _ => ErrorKind::NotFound,
    })?;
    let density_byte = (jedec_id & 0xFF) as u8;

    // Capacity = 2^(density_byte) bytes
    // Guard against overflow: use u64 shift for density >= 31
    let capacity_bytes: u64 = if density_byte < 31 {
        (1u32 << density_byte) as u64
    } else {
        1u64 << density_byte
    };

    let block_driver = SpiFlashBlockDriver::new(flash_cmd, capacity_bytes);

    let block = Block::<BlockError<FlashBlockError>, { FLASH_SECTOR_SIZE as usize }>::new(
        FLASH_STORAGE_NAME,
        Arc::new(SpinLock::new(block_driver)),
    );

    DeviceManager::get()
        .register_device(String::from(FLASH_STORAGE_NAME), Arc::new(block))
        .map_err(|_| ErrorKind::AlreadyExists)?;

    Ok(())
}

// SpiFlashBlockDriver is accessed exclusively through SpinLock, which guarantees
// mutual exclusion. SPI must be Send + Sync for the unsafe impl to be sound:
// Send — safe to transfer across threads (required by SpinLock);
// Sync — safe to share &self across threads (required by BlockDriverOps: Send + Sync).

unsafe impl<SPI: SpiDevice<u8> + Send + Sync> Sync for SpiFlashBlockDriver<SPI> {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::block::{Block, BlockDriverOps, BlockError, ErrorType};
    use alloc::sync::Arc;
    use core::cell::UnsafeCell;
    use embedded_hal::spi::{ErrorKind, Operation, SpiDevice};

    // Reuse MockSpiDevice pattern from spi_flash_cmd tests
    struct MockSpiDevice {
        shared: Arc<UnsafeCell<MockSpiShared>>,
    }

    struct MockSpiShared {
        /// All write data from Write operations, concatenated
        writes: alloc::vec::Vec<u8>,
        /// Number of DelayNs operations seen
        delays: usize,
        /// Data to return on Read operations (queue-style: consumed per read)
        read_queue: alloc::vec::Vec<alloc::vec::Vec<u8>>,
        /// Whether the next transaction should fail
        should_fail: bool,
        /// Total transaction count
        transaction_count: usize,
    }

    unsafe impl Send for MockSpiDevice {}
    unsafe impl Sync for MockSpiDevice {}

    #[derive(Debug, Clone, Copy)]
    struct MockSpiError;

    impl embedded_hal::spi::ErrorType for MockSpiDevice {
        type Error = MockSpiError;
    }

    impl embedded_hal::spi::Error for MockSpiError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl SpiDevice<u8> for MockSpiDevice {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            let shared = self.shared.get();
            // SAFETY: Single-threaded test context, accessed exclusively
            let shared = unsafe { &mut *shared };
            shared.transaction_count += 1;

            if shared.should_fail {
                shared.should_fail = false;
                return Err(MockSpiError);
            }

            for op in operations.iter_mut() {
                match op {
                    Operation::Write(data) => {
                        shared.writes.extend_from_slice(data);
                    }
                    Operation::Read(buf) => {
                        if !shared.read_queue.is_empty() {
                            let data = &shared.read_queue[0];
                            let len = buf.len().min(data.len());
                            buf[..len].copy_from_slice(&data[..len]);
                            shared.read_queue.remove(0);
                        }
                    }
                    Operation::Transfer(read_buf, write_buf) => {
                        shared.writes.extend_from_slice(write_buf);
                        if !shared.read_queue.is_empty() {
                            let data = &shared.read_queue[0];
                            let len = read_buf.len().min(data.len());
                            read_buf[..len].copy_from_slice(&data[..len]);
                            shared.read_queue.remove(0);
                        }
                    }
                    Operation::TransferInPlace(buf) => {
                        shared.writes.extend_from_slice(buf);
                    }
                    Operation::DelayNs(_) => {
                        shared.delays += 1;
                    }
                }
            }
            Ok(())
        }
    }

    impl MockSpiDevice {
        fn new(shared: Arc<UnsafeCell<MockSpiShared>>) -> Self {
            MockSpiDevice { shared }
        }
    }

    impl MockSpiShared {
        fn new() -> Self {
            MockSpiShared {
                writes: alloc::vec::Vec::new(),
                delays: 0,
                read_queue: alloc::vec::Vec::new(),
                should_fail: false,
                transaction_count: 0,
            }
        }
    }

    fn with_shared<R>(
        shared: &Arc<UnsafeCell<MockSpiShared>>,
        f: impl FnOnce(&mut MockSpiShared) -> R,
    ) -> R {
        // SAFETY: Test-only, single-threaded context
        f(unsafe { &mut *shared.get() })
    }

    /// Create a SpiFlashBlockDriver with a MockSpiDevice
    ///
    /// Returns (block_driver, shared_state) so tests can configure mock
    /// responses and inspect recorded operations.
    fn create_block_driver(
        capacity_bytes: u64,
    ) -> (
        SpiFlashBlockDriver<MockSpiDevice>,
        Arc<UnsafeCell<MockSpiShared>>,
    ) {
        let shared = Arc::new(UnsafeCell::new(MockSpiShared::new()));
        let mock = MockSpiDevice::new(Arc::clone(&shared));
        let flash_cmd = SpiFlashCmd::new(mock);
        let driver = SpiFlashBlockDriver::new(flash_cmd, capacity_bytes);
        (driver, shared)
    }

    #[test]
    fn test_block_driver_capacity() {
        let (driver, _shared) = create_block_driver(1024 * 1024); // 1MB
        assert_eq!(driver.capacity(), 1024 * 1024 / 512); // 2048 sectors
        assert_eq!(driver.sector_size(), 512);
    }

    #[test]
    fn test_read_blocks_from_flash() {
        let (mut driver, shared) = create_block_driver(1024 * 1024);
        let mut buf = [0u8; 512];

        // Provide read data for a 512-byte read
        with_shared(&shared, |s| {
            s.read_queue.push(alloc::vec![0xAA; 512]);
        });

        driver.read_blocks(0, &mut buf).unwrap();
        assert_eq!(buf[0], 0xAA);

        // Verify the read command was sent
        with_shared(&shared, |s| {
            assert_eq!(s.writes[0], 0x03); // READ command
        });
    }

    #[test]
    fn test_read_blocks_from_dirty_cache() {
        let (mut driver, shared) = create_block_driver(1024 * 1024);
        let mut write_buf = [0xBB; 512];

        // Write to block 0 — marks cache dirty
        // This triggers ensure_erase_block → read_erase_block
        with_shared(&shared, |s| {
            // First: read_erase_block reads 4096 bytes
            s.read_queue.push(alloc::vec![0u8; FLASH_ERASE_SIZE]);
        });
        driver.write_blocks(0, &write_buf).unwrap();

        // Now read from the same erase block — should come from cache
        let mut read_buf = [0u8; 512];
        // Reset write tracking to verify no new SPI transactions for the read
        with_shared(&shared, |s| {
            s.writes.clear();
            s.transaction_count = 0;
        });
        driver.read_blocks(0, &mut read_buf).unwrap();
        assert_eq!(read_buf[0], 0xBB); // Data from cache, not from SPI

        // No new SPI transactions should have occurred
        with_shared(&shared, |s| {
            assert_eq!(s.transaction_count, 0);
        });
    }

    #[test]
    fn test_write_marks_dirty() {
        let (mut driver, shared) = create_block_driver(1024 * 1024);
        let write_data = [0xCC; 512];

        // Provide data for read_erase_block
        with_shared(&shared, |s| {
            s.read_queue.push(alloc::vec![0u8; FLASH_ERASE_SIZE]);
        });

        driver.write_blocks(0, &write_data).unwrap();

        // Driver should be dirty but not flushed
        assert!(driver.dirty);
        assert_eq!(driver.current_erase_block, Some(0));
    }

    #[test]
    fn test_flush_erase_block() {
        let (mut driver, shared) = create_block_driver(1024 * 1024);
        let write_data = [0xDD; 512];

        // Setup: read_erase_block needs data, write_enable + wait_busy need responses
        with_shared(&shared, |s| {
            // read_erase_block: 4096 bytes of zeros
            s.read_queue.push(alloc::vec![0u8; FLASH_ERASE_SIZE]);
            // After write_enable: status with WEL bit (0x02)
            s.read_queue.push(alloc::vec![0x02]);
            // After sector_erase: wait_busy status (0x00 = not busy)
            s.read_queue.push(alloc::vec![0x00]);
            // page_program write_enable: status with WEL bit (0x02)
            s.read_queue.push(alloc::vec![0x02]);
            // page_program wait_busy: status (0x00)
            s.read_queue.push(alloc::vec![0x00]);
        });

        driver.write_blocks(0, &write_data).unwrap();
        driver.flush().unwrap();

        // Verify dirty flag cleared after flush
        assert!(!driver.dirty);
    }

    #[test]
    fn test_ensure_erase_block_switching() {
        let (mut driver, shared) = create_block_driver(1024 * 1024);
        // Block 0 and block 8 are in different erase blocks
        // (erase block 0 = blocks 0-7, erase block 1 = blocks 8-15)

        // Write to block 0 — caches erase block 0
        with_shared(&shared, |s| {
            s.read_queue.push(alloc::vec![0u8; FLASH_ERASE_SIZE]);
            // For flush: write_enable + wait_busy responses
            s.read_queue.push(alloc::vec![0x02]); // WEL
            s.read_queue.push(alloc::vec![0x00]); // not busy
                                                  // For new read_erase_block
            s.read_queue.push(alloc::vec![0xFF; FLASH_ERASE_SIZE]);
        });

        driver.write_blocks(0, &[0xAA; 512]).unwrap();
        assert_eq!(driver.current_erase_block, Some(0));

        // Write to block 8 — should flush block 0, then cache block 1
        driver.write_blocks(8, &[0xBB; 512]).unwrap();
        assert_eq!(driver.current_erase_block, Some(1));
    }

    #[test]
    fn test_block_offset_in_erase() {
        let (driver, _shared) = create_block_driver(1024 * 1024);
        // Block 0 in erase block 0 → offset 0
        assert_eq!(driver.block_offset_in_erase(0), 0);
        // Block 7 in erase block 0 → offset 7 * 512 = 3584
        assert_eq!(driver.block_offset_in_erase(7), 3584);
        // Block 8 in erase block 1 → offset 0
        assert_eq!(driver.block_offset_in_erase(8), 0);
        // Block 9 in erase block 1 → offset 1 * 512 = 512
        assert_eq!(driver.block_offset_in_erase(9), 512);
    }

    #[test]
    fn test_flush_no_dirty_no_op() {
        let (mut driver, shared) = create_block_driver(1024 * 1024);
        // No writes performed — dirty = false, current_erase_block = None
        driver.flush().unwrap();

        // No SPI transactions should have occurred for flush
        with_shared(&shared, |s| {
            assert_eq!(s.transaction_count, 0);
        });
    }

    #[test]
    fn test_spi_error_on_read() {
        let (mut driver, shared) = create_block_driver(1024 * 1024);
        let mut buf = [0u8; 512];

        with_shared(&shared, |s| {
            s.should_fail = true;
        });

        let result = driver.read_blocks(0, &mut buf);
        assert!(result.is_err());
    }

    #[test]
    fn test_block_error_kind_mapping() {
        // Verify BlockError<FlashBlockError> → embedded_io ErrorKind mapping
        use embedded_io::Error as IOError;

        // FlashError::Spi → Other
        let err = BlockError::Driver(FlashBlockError::Flash(
            crate::devices::storage::spi_flash_cmd::FlashError::Spi(ErrorKind::Other),
        ));
        assert_eq!(IOError::kind(&err), embedded_io::ErrorKind::Other);

        // FlashError::NotReady → Other
        let err = BlockError::Driver(FlashBlockError::Flash(
            crate::devices::storage::spi_flash_cmd::FlashError::NotReady,
        ));
        assert_eq!(IOError::kind(&err), embedded_io::ErrorKind::Other);

        // FlashError::Timeout → TimedOut
        let err = BlockError::Driver(FlashBlockError::Flash(
            crate::devices::storage::spi_flash_cmd::FlashError::Timeout,
        ));
        assert_eq!(IOError::kind(&err), embedded_io::ErrorKind::TimedOut);

        // FlashError::AddrOverflow → InvalidInput
        let err = BlockError::Driver(FlashBlockError::Flash(
            crate::devices::storage::spi_flash_cmd::FlashError::AddrOverflow { addr: 0x1000000 },
        ));
        assert_eq!(IOError::kind(&err), embedded_io::ErrorKind::InvalidInput);

        // FlashError::JedecMismatch → NotFound
        let err = BlockError::Driver(FlashBlockError::Flash(
            crate::devices::storage::spi_flash_cmd::FlashError::JedecMismatch {
                expected: 0xEF4018,
                got: 0x000000,
            },
        ));
        assert_eq!(IOError::kind(&err), embedded_io::ErrorKind::NotFound);

        // FlashError::WriteEnableFailed → Other
        let err = BlockError::Driver(FlashBlockError::Flash(
            crate::devices::storage::spi_flash_cmd::FlashError::WriteEnableFailed,
        ));
        assert_eq!(IOError::kind(&err), embedded_io::ErrorKind::Other);

        // FlashError::InvalidParam → InvalidInput
        let err = BlockError::Driver(FlashBlockError::Flash(
            crate::devices::storage::spi_flash_cmd::FlashError::InvalidParam("test"),
        ));
        assert_eq!(IOError::kind(&err), embedded_io::ErrorKind::InvalidInput);
    }
}
