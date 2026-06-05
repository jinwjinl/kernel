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

//! JEDEC 25-series SPI NOR Flash command layer
//!
//! Generic command interface for standard SPI NOR Flash chips (JEDEC 25-series).
//! Operates via `embedded_hal::spi::SpiDevice<u8>` to send read, erase, and
//! program commands.

use embedded_hal::spi::{ErrorKind, Operation, SpiDevice};

/// SPI NOR Flash command layer error
#[derive(Debug, Clone, Eq, PartialEq, thiserror::Error)]
pub enum FlashError {
    /// SPI bus communication error
    #[error("SPI bus error: {0:?}")]
    Spi(ErrorKind),
    /// Device not ready for operation
    #[error("Device not ready")]
    NotReady,
    /// JEDEC ID does not match expected value
    #[error("JEDEC ID mismatch: expected 0x{expected:06X}, got 0x{got:06X}")]
    JedecMismatch { expected: u32, got: u32 },
    /// Write-enable failed (WEL bit not set after command)
    #[error("Write enable failed")]
    WriteEnableFailed,
    /// Operation timed out (busy polling limit exceeded)
    #[error("Timeout")]
    Timeout,
    /// Address exceeds 24-bit range (>= 16MB), requires 4-byte addressing mode
    #[error("Address 0x{addr:08X} exceeds 24-bit range")]
    AddrOverflow { addr: u32 },
    /// Invalid parameter (data too long for page program, etc.)
    #[error("Invalid parameter: {0}")]
    InvalidParam(&'static str),
}

/// Convert an SpiDevice error into a FlashError by extracting its ErrorKind
///
/// This is used inline (as a closure) rather than as a `From` impl to avoid
/// Rust coherence issues with generic `SPI::Error` type parameters.
fn spi_err_to_flash<E: embedded_hal::spi::Error>(err: E) -> FlashError {
    FlashError::Spi(err.kind())
}

/// JEDEC 25-series SPI NOR Flash command layer
///
/// Provides low-level read, erase, and program operations on standard SPI NOR
/// Flash chips via `embedded_hal::spi::SpiDevice<u8>` transaction operations.
pub struct SpiFlashCmd<SPI: SpiDevice<u8>> {
    spi: SPI,
}

impl<SPI: SpiDevice<u8>> SpiFlashCmd<SPI> {
    /// Create a new SPI Flash command layer from an SpiDevice
    pub fn new(spi: SPI) -> Self {
        SpiFlashCmd { spi }
    }

    /// Read the JEDEC manufacturer + device ID (3 bytes, MSB-first)
    ///
    /// Returns `manuf << 16 | type << 8 | density` as a u32.
    /// Command: 0x9F
    pub fn jedec_id(&mut self) -> Result<u32, FlashError> {
        let mut id_buf = [0u8; 3];
        self.spi
            .transaction(&mut [Operation::Write(&[0x9F]), Operation::Read(&mut id_buf)])
            .map_err(spi_err_to_flash)?;
        Ok((id_buf[0] as u32) << 16 | (id_buf[1] as u32) << 8 | (id_buf[2] as u32))
    }

    /// Normal read (3-byte address, no dummy cycles)
    ///
    /// Command: 0x03
    pub fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), FlashError> {
        let addr_bytes = addr_bytes(addr)?;
        self.spi
            .transaction(&mut [
                Operation::Write(&[0x03, addr_bytes[0], addr_bytes[1], addr_bytes[2]]),
                Operation::Read(buf),
            ])
            .map_err(spi_err_to_flash)?;
        Ok(())
    }

    /// Fast read (3-byte address + 8 dummy cycles)
    ///
    /// Command: 0x0B
    /// For simplicity, this sends the command without explicit dummy cycles.
    /// The SPI controller must handle dummy cycles if configured, or use
    /// normal read (0x03) instead.
    pub fn fast_read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), FlashError> {
        let addr_bytes = addr_bytes(addr)?;
        self.spi
            .transaction(&mut [
                Operation::Write(&[0x0B, addr_bytes[0], addr_bytes[1], addr_bytes[2]]),
                Operation::DelayNs(1_000_000), // ~8 dummy clock cycles at typical SPI speeds
                Operation::Read(buf),
            ])
            .map_err(spi_err_to_flash)?;
        Ok(())
    }

    /// Page program (auto write-enable first)
    ///
    /// Command: 0x02
    /// Data must be <= 256 bytes. Address must be aligned to a 256-byte page boundary.
    pub fn page_program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashError> {
        if addr % 256 != 0 {
            return Err(FlashError::InvalidParam(
                "page_program address not 256-byte aligned",
            ));
        }
        if data.len() > 256 {
            return Err(FlashError::InvalidParam(
                "page_program data exceeds 256 bytes",
            ));
        }
        self.write_enable()?;
        let addr_bytes = addr_bytes(addr)?;
        self.spi
            .transaction(&mut [
                Operation::Write(&[0x02, addr_bytes[0], addr_bytes[1], addr_bytes[2]]),
                Operation::Write(data),
            ])
            .map_err(spi_err_to_flash)?;
        self.wait_busy()?;
        Ok(())
    }

    /// 4KB sector erase (auto write-enable, auto wait_busy)
    ///
    /// Command: 0x20
    pub fn sector_erase(&mut self, addr: u32) -> Result<(), FlashError> {
        self.write_enable()?;
        let addr_bytes = addr_bytes(addr)?;
        self.spi
            .transaction(&mut [Operation::Write(&[
                0x20,
                addr_bytes[0],
                addr_bytes[1],
                addr_bytes[2],
            ])])
            .map_err(spi_err_to_flash)?;
        self.wait_busy()?;
        Ok(())
    }

    /// 32KB block erase (auto write-enable, auto wait_busy)
    ///
    /// Command: 0x52
    pub fn block_erase_32k(&mut self, addr: u32) -> Result<(), FlashError> {
        self.write_enable()?;
        let addr_bytes = addr_bytes(addr)?;
        self.spi
            .transaction(&mut [Operation::Write(&[
                0x52,
                addr_bytes[0],
                addr_bytes[1],
                addr_bytes[2],
            ])])
            .map_err(spi_err_to_flash)?;
        self.wait_busy()?;
        Ok(())
    }

    /// 64KB block erase (auto write-enable, auto wait_busy)
    ///
    /// Command: 0xD8
    pub fn block_erase_64k(&mut self, addr: u32) -> Result<(), FlashError> {
        self.write_enable()?;
        let addr_bytes = addr_bytes(addr)?;
        self.spi
            .transaction(&mut [Operation::Write(&[
                0xD8,
                addr_bytes[0],
                addr_bytes[1],
                addr_bytes[2],
            ])])
            .map_err(spi_err_to_flash)?;
        self.wait_busy()?;
        Ok(())
    }

    /// Full chip erase (auto write-enable, auto wait_busy)
    ///
    /// Command: 0xC7
    pub fn chip_erase(&mut self) -> Result<(), FlashError> {
        self.write_enable()?;
        self.spi
            .transaction(&mut [Operation::Write(&[0xC7])])
            .map_err(spi_err_to_flash)?;
        self.wait_busy()?;
        Ok(())
    }

    /// Set Write Enable Latch (WEL) bit
    ///
    /// Command: 0x06
    /// Verifies that WEL bit (bit 1) is set in the status register after
    /// sending the command.
    pub fn write_enable(&mut self) -> Result<(), FlashError> {
        self.spi
            .transaction(&mut [Operation::Write(&[0x06])])
            .map_err(spi_err_to_flash)?;
        // Verify WEL bit is set (bit 1 of status register)
        let status = self.read_status()?;
        if status & 0x02 == 0 {
            return Err(FlashError::WriteEnableFailed);
        }
        Ok(())
    }

    /// Read status register byte
    ///
    /// Command: 0x05
    pub fn read_status(&mut self) -> Result<u8, FlashError> {
        let mut status_buf = [0u8; 1];
        self.spi
            .transaction(&mut [Operation::Write(&[0x05]), Operation::Read(&mut status_buf)])
            .map_err(spi_err_to_flash)?;
        Ok(status_buf[0])
    }

    /// Wait until BUSY bit (bit 0) clears
    ///
    /// Polls the status register with 1ms delays between reads.
    /// Returns `FlashError::Timeout` if busy persists after 1000 iterations.
    pub fn wait_busy(&mut self) -> Result<(), FlashError> {
        for _ in 0..1000 {
            let status = self.read_status()?;
            if status & 0x01 == 0 {
                return Ok(());
            }
            self.spi
                .transaction(&mut [Operation::DelayNs(1_000_000)])
                .map_err(spi_err_to_flash)?;
        }
        Err(FlashError::Timeout)
    }

    /// Release from deep power-down
    ///
    /// Command: 0xAB
    pub fn release_from_deep_power_down(&mut self) -> Result<(), FlashError> {
        self.spi
            .transaction(&mut [Operation::Write(&[0xAB])])
            .map_err(spi_err_to_flash)?;
        Ok(())
    }
}

/// Helper: split a 24-bit address into 3 bytes (MSB-first)
///
/// Returns `Err(FlashError::AddrOverflow)` if the address exceeds the 24-bit
/// range (>= 0x01000000). Flash chips larger than 16MB require 4-byte addressing.
fn addr_bytes(addr: u32) -> Result<[u8; 3], FlashError> {
    if addr >= 0x0100_0000 {
        return Err(FlashError::AddrOverflow { addr });
    }
    Ok([
        ((addr >> 16) & 0xFF) as u8,
        ((addr >> 8) & 0xFF) as u8,
        (addr & 0xFF) as u8,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::sync::Arc;
    use core::cell::UnsafeCell;

    /// Mock SPI device for testing SpiFlashCmd
    ///
    /// Records all transaction operations for verification and returns
    /// configurable preset data for Read operations.
    struct MockSpiDevice {
        shared: Arc<UnsafeCell<MockSpiShared>>,
    }

    /// Shared state between MockSpiDevice instances (allows interior mutability
    /// through UnsafeCell since tests run single-threaded under SpinLock)
    struct MockSpiShared {
        /// All write data from Write operations, concatenated
        writes: alloc::vec::Vec<u8>,
        /// Number of DelayNs operations seen
        delays: usize,
        /// Per-transaction read data (consumed in order by Read operations)
        read_queue: alloc::vec::Vec<alloc::vec::Vec<u8>>,
        /// Whether the next transaction should fail
        should_fail: bool,
        /// Number of transactions executed
        transaction_count: usize,
    }

    // MockSpiDevice is Send+Sync: accessed exclusively through SpinLock which
    // guarantees mutual exclusion. UnsafeCell is safe because the driver only
    // accesses it while holding the SpinLock.
    unsafe impl Send for MockSpiDevice {}
    unsafe impl Sync for MockSpiDevice {}

    impl embedded_hal::spi::ErrorType for MockSpiDevice {
        type Error = MockSpiError;
    }

    #[derive(Debug, Clone, Copy)]
    struct MockSpiError;

    impl embedded_hal::spi::Error for MockSpiError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl embedded_hal::spi::SpiDevice<u8> for MockSpiDevice {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            let shared = self.shared.get();
            // SAFETY: MockSpiDevice is always wrapped in SpinLock which guarantees
            // exclusive access. No concurrent access possible.
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
                            let read_data = &shared.read_queue[0];
                            let len = buf.len().min(read_data.len());
                            buf[..len].copy_from_slice(&read_data[..len]);
                            // If this Read consumed all data from the current queue entry, pop it
                            if read_data.len() <= buf.len() {
                                shared.read_queue.remove(0);
                            }
                        }
                    }
                    Operation::Transfer(read_buf, write_buf) => {
                        shared.writes.extend_from_slice(write_buf);
                        if !shared.read_queue.is_empty() {
                            let read_data = &shared.read_queue[0];
                            let len = read_buf.len().min(read_data.len());
                            read_buf[..len].copy_from_slice(&read_data[..len]);
                            if read_data.len() <= read_buf.len() {
                                shared.read_queue.remove(0);
                            }
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

    /// Helper to create a test SpiFlashCmd with a MockSpiDevice
    ///
    /// Returns (flash_cmd, shared_state) so tests can inspect recorded operations.
    fn create_flash_cmd() -> (SpiFlashCmd<MockSpiDevice>, Arc<UnsafeCell<MockSpiShared>>) {
        let shared = Arc::new(UnsafeCell::new(MockSpiShared::new()));
        let mock = MockSpiDevice::new(Arc::clone(&shared));
        let flash_cmd = SpiFlashCmd::new(mock);
        (flash_cmd, shared)
    }

    /// Access shared state mutably (safe in single-threaded test context)
    fn with_shared<R>(
        shared: &Arc<UnsafeCell<MockSpiShared>>,
        f: impl FnOnce(&mut MockSpiShared) -> R,
    ) -> R {
        // SAFETY: Test-only, single-threaded context
        f(unsafe { &mut *shared.get() })
    }

    #[test]
    fn test_addr_bytes_valid() {
        // Zero address
        let result = addr_bytes(0x000000);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), [0x00, 0x00, 0x00]);

        // Max valid 24-bit address (0x00FFFFFF)
        let result = addr_bytes(0x00FFFFFF);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), [0xFF, 0xFF, 0xFF]);

        // Typical address
        let result = addr_bytes(0x001234);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), [0x00, 0x12, 0x34]);
    }

    #[test]
    fn test_addr_bytes_overflow() {
        let result = addr_bytes(0x01000000);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            FlashError::AddrOverflow { addr: 0x01000000 }
        );

        let result = addr_bytes(0xFFFFFFFF);
        assert!(result.is_err());
    }

    #[test]
    fn test_jedec_id() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        // Set read queue: manufacturer=0xEF, type=0x40, density=0x18
        // This simulates a Winbond W25Q16 (16MB)
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0xEF, 0x40, 0x18]];
        });

        let jedec_id = flash_cmd.jedec_id().unwrap();
        assert_eq!(jedec_id, 0xEF4018);

        // Verify command was sent: jedec_id writes [0x9F] then reads 3 bytes
        with_shared(&shared, |s| {
            assert_eq!(&s.writes, &[0x9F]); // JEDEC ID command
            assert_eq!(s.transaction_count, 1);
        });
    }

    #[test]
    fn test_read_command() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        let mut buf = [0u8; 4];
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0xAA, 0xBB, 0xCC, 0xDD]];
        });

        flash_cmd.read(0x001000, &mut buf).unwrap();
        assert_eq!(buf, [0xAA, 0xBB, 0xCC, 0xDD]);

        with_shared(&shared, |s| {
            // Verify command byte (0x03) + 3 address bytes
            assert_eq!(&s.writes, &[0x03, 0x00, 0x10, 0x00]);
        });
    }

    #[test]
    fn test_read_addr_overflow() {
        let (mut flash_cmd, _shared) = create_flash_cmd();
        let mut buf = [0u8; 4];
        let result = flash_cmd.read(0x01000000, &mut buf);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            FlashError::AddrOverflow { addr: 0x01000000 }
        );
    }

    #[test]
    fn test_fast_read_command() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        let mut buf = [0u8; 4];
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0x11, 0x22, 0x33, 0x44]];
        });

        flash_cmd.fast_read(0x000100, &mut buf).unwrap();
        assert_eq!(buf, [0x11, 0x22, 0x33, 0x44]);

        with_shared(&shared, |s| {
            // Verify command byte (0x0B) + 3 address bytes
            assert_eq!(&s.writes[..4], &[0x0B, 0x00, 0x01, 0x00]);
            // Fast read includes a DelayNs for dummy cycles
            assert!(s.delays > 0);
        });
    }

    #[test]
    fn test_page_program_param_validation() {
        let (mut flash_cmd, _shared) = create_flash_cmd();
        let data = [0u8; 128];

        // Address not page-aligned
        let result = flash_cmd.page_program(0x001001, &data);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            FlashError::InvalidParam("page_program address not 256-byte aligned")
        );

        // Data exceeds 256 bytes
        let big_data = alloc::vec![0u8; 300];
        let result = flash_cmd.page_program(0x000000, &big_data);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            FlashError::InvalidParam("page_program data exceeds 256 bytes")
        );
    }

    #[test]
    fn test_page_program_success() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        // Configure mock: write_enable succeeds, wait_busy succeeds
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0x02], alloc::vec![0x00]];
        });

        let data = [0xAA, 0xBB, 0xCC, 0xDD];
        flash_cmd.page_program(0x000000, &data).unwrap();

        with_shared(&shared, |s| {
            // Verify the full write sequence:
            // write_enable: [0x06], read_status: [0x05],
            // page_program command: [0x02, 0x00, 0x00, 0x00], data: [0xAA, 0xBB, 0xCC, 0xDD],
            // wait_busy: read_status: [0x05]
            let writes = &s.writes;
            assert_eq!(writes[0], 0x06); // WREN
            assert_eq!(writes[1], 0x05); // read_status in write_enable
            assert_eq!(&writes[2..6], &[0x02, 0x00, 0x00, 0x00]); // PP command + addr
            assert_eq!(&writes[6..10], &[0xAA, 0xBB, 0xCC, 0xDD]); // data bytes
            assert_eq!(writes[10], 0x05); // read_status in wait_busy
        });
    }

    #[test]
    fn test_sector_erase_command() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        // Configure mock: write_enable succeeds (WEL bit set), wait_busy succeeds
        with_shared(&shared, |s| {
            // Queue entry 1: write_enable read_status returns WEL=1 (0x02)
            // Queue entry 2: wait_busy read_status returns not-busy (0x00)
            s.read_queue = alloc::vec![alloc::vec![0x02], alloc::vec![0x00]];
        });

        flash_cmd.sector_erase(0x001000).unwrap();

        with_shared(&shared, |s| {
            // Verify the full write sequence:
            // write_enable: [0x06], read_status: [0x05],
            // sector_erase: [0x20, 0x00, 0x10, 0x00],
            // wait_busy: read_status: [0x05]
            let writes = &s.writes;
            assert_eq!(writes[0], 0x06); // WREN
            assert_eq!(writes[1], 0x05); // read_status in write_enable
            assert_eq!(writes[2], 0x20); // Sector erase command
            assert_eq!(&writes[3..6], &[0x00, 0x10, 0x00]); // Address bytes
            assert_eq!(writes[6], 0x05); // read_status in wait_busy
        });
    }

    #[test]
    fn test_write_enable_success() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        // Status register with WEL bit set (bit 1 = 0x02)
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0x02]];
        });

        flash_cmd.write_enable().unwrap();

        with_shared(&shared, |s| {
            assert_eq!(s.writes[0], 0x06); // WREN command
            assert_eq!(s.writes[1], 0x05); // read_status command
        });
    }

    #[test]
    fn test_write_enable_failed() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        // Status register without WEL bit (0x00)
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0x00]];
        });

        let result = flash_cmd.write_enable();
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), FlashError::WriteEnableFailed);
    }

    #[test]
    fn test_read_status() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0x03]]; // BUSY + WEL
        });

        let status = flash_cmd.read_status().unwrap();
        assert_eq!(status, 0x03);

        with_shared(&shared, |s| {
            assert_eq!(s.writes[0], 0x05); // RDSR command
        });
    }

    #[test]
    fn test_chip_erase_command() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        with_shared(&shared, |s| {
            // write_enable succeeds, wait_busy succeeds
            s.read_queue = alloc::vec![alloc::vec![0x02], alloc::vec![0x00]];
        });

        flash_cmd.chip_erase().unwrap();

        with_shared(&shared, |s| {
            // Verify the full write sequence:
            // write_enable: [0x06], read_status: [0x05],
            // chip_erase: [0xC7],
            // wait_busy: read_status: [0x05]
            assert_eq!(s.writes[0], 0x06); // WREN
            assert_eq!(s.writes[1], 0x05); // read_status in write_enable
            assert_eq!(s.writes[2], 0xC7); // Chip erase command
            assert_eq!(s.writes[3], 0x05); // read_status in wait_busy
        });
    }

    #[test]
    fn test_release_from_deep_power_down() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        flash_cmd.release_from_deep_power_down().unwrap();

        with_shared(&shared, |s| {
            assert_eq!(&s.writes, &[0xAB]);
        });
    }

    #[test]
    fn test_block_erase_32k_command() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        with_shared(&shared, |s| {
            // write_enable succeeds, wait_busy succeeds
            s.read_queue = alloc::vec![alloc::vec![0x02], alloc::vec![0x00]];
        });

        flash_cmd.block_erase_32k(0x001000).unwrap();

        with_shared(&shared, |s| {
            // Verify the full write sequence:
            // write_enable: [0x06], read_status: [0x05],
            // block_erase_32k: [0x52, 0x00, 0x10, 0x00],
            // wait_busy: read_status: [0x05]
            let writes = &s.writes;
            assert_eq!(writes[0], 0x06); // WREN
            assert_eq!(writes[1], 0x05); // read_status in write_enable
            assert_eq!(writes[2], 0x52); // 32KB block erase command
            assert_eq!(&writes[3..6], &[0x00, 0x10, 0x00]); // Address bytes
            assert_eq!(writes[6], 0x05); // read_status in wait_busy
        });
    }

    #[test]
    fn test_block_erase_64k_command() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        with_shared(&shared, |s| {
            // write_enable succeeds, wait_busy succeeds
            s.read_queue = alloc::vec![alloc::vec![0x02], alloc::vec![0x00]];
        });

        flash_cmd.block_erase_64k(0x001000).unwrap();

        with_shared(&shared, |s| {
            // Verify the full write sequence:
            // write_enable: [0x06], read_status: [0x05],
            // block_erase_64k: [0xD8, 0x00, 0x10, 0x00],
            // wait_busy: read_status: [0x05]
            let writes = &s.writes;
            assert_eq!(writes[0], 0x06); // WREN
            assert_eq!(writes[1], 0x05); // read_status in write_enable
            assert_eq!(writes[2], 0xD8); // 64KB block erase command
            assert_eq!(&writes[3..6], &[0x00, 0x10, 0x00]); // Address bytes
            assert_eq!(writes[6], 0x05); // read_status in wait_busy
        });
    }

    #[test]
    fn test_wait_busy_timeout() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        // Simulate device staying busy: all read_status calls return busy (0x01)
        // wait_busy polls up to 1000 iterations, each needs a read_status entry
        with_shared(&shared, |s| {
            s.read_queue = alloc::vec![alloc::vec![0x01]; 1000];
        });

        let result = flash_cmd.wait_busy();
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), FlashError::Timeout);

        // Verify that all 1000 iterations were executed
        with_shared(&shared, |s| {
            // Each iteration: read_status (1 transaction) + DelayNs (1 transaction)
            assert_eq!(s.transaction_count, 2000);
            assert_eq!(s.delays, 1000);
        });
    }

    #[test]
    fn test_spi_error_propagation() {
        let (mut flash_cmd, shared) = create_flash_cmd();
        with_shared(&shared, |s| {
            s.should_fail = true;
        });

        let result = flash_cmd.jedec_id();
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), FlashError::Spi(ErrorKind::Other));
    }
}
