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

//! SPI block transport bridge — HAL `Spi` trait → `embedded_hal::spi::SpiDevice<u8>`
//!
//! Unlike BlockI2c (BusWrapper/BusInterface), BlockSpi directly implements
//! `SpiDevice<u8>` since SPI NOR Flash drivers consume SpiDevice directly.

use blueos_driver::spi::SpiConfig;
use blueos_hal::PlatPeri;
use embedded_hal::{delay::DelayNs, spi::Operation};

/// SPI block transport wrapper
///
/// Wraps a HAL `Spi` peripheral into an `embedded_hal::spi::SpiDevice<u8>`,
/// enabling SPI device drivers (like NOR Flash) to use standard embedded-hal
/// transaction operations.
pub struct BlockSpi<T: PlatPeri> {
    inner: &'static T,
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>> BlockSpi<T> {
    /// Create a new BlockSpi, configuring the underlying HAL peripheral
    /// with SPI NOR Flash default settings (Mode 0, 20MHz, MSB-first).
    pub fn new(inner: &'static T) -> Result<Self, blueos_hal::err::HalError> {
        inner.configure(&SpiConfig::spi_flash_default())?;
        Ok(BlockSpi { inner })
    }
}

// Error type for SpiDevice implementation
impl<T: blueos_hal::spi::Spi<SpiConfig, ()>> embedded_hal::spi::ErrorType for BlockSpi<T> {
    type Error = SpiBlockError;
}

/// SPI block transport error type
///
/// Maps HAL errors and invalid operation conditions to embedded-hal's
/// SPI error framework.
#[derive(Debug, Clone, Copy)]
pub enum SpiBlockError {
    /// Error originating from the underlying HAL Spi peripheral
    HalError,
    /// An invalid or unsupported SPI operation was requested
    InvalidOperation,
}

impl embedded_hal::spi::Error for SpiBlockError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            SpiBlockError::HalError => embedded_hal::spi::ErrorKind::Other,
            SpiBlockError::InvalidOperation => embedded_hal::spi::ErrorKind::Other,
        }
    }
}

/// SpiDevice<u8> implementation — bridges HAL Spi to embedded-hal SpiDevice
///
/// This implementation translates embedded-hal SPI transaction operations
/// (Read, Write, Transfer, TransferInPlace, DelayNs) into calls on the
/// underlying HAL `Spi` trait methods (read, write, transfer).
///
/// # CS (Chip Select) management
///
/// TODO: GPIO bit-bang CS management — assert CS before the first operation
/// in a transaction and deassert CS after the last operation. The ESP32-C3
/// board will handle CS GPIO directly in Step 8 using SpiConfig.cs_pin.
/// Until then, CS must be managed externally (e.g., by hardware or board setup).
impl<T: blueos_hal::spi::Spi<SpiConfig, ()>> embedded_hal::spi::SpiDevice<u8> for BlockSpi<T> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        for op in operations.iter_mut() {
            match op {
                Operation::Write(write_buf) => {
                    self.inner
                        .write(write_buf)
                        .map_err(|_| SpiBlockError::HalError)?;
                }
                Operation::Read(read_buf) => {
                    self.inner
                        .read(read_buf)
                        .map_err(|_| SpiBlockError::HalError)?;
                }
                Operation::Transfer(read_buf, write_buf) => {
                    self.inner
                        .transfer(read_buf, write_buf)
                        .map_err(|_| SpiBlockError::HalError)?;
                }
                Operation::TransferInPlace(buf) => {
                    // Write the buf contents, then read back into the same buf
                    // (two-step since transfer(read, write) can't alias the same buffer)
                    self.inner.write(buf).map_err(|_| SpiBlockError::HalError)?;
                    self.inner.read(buf).map_err(|_| SpiBlockError::HalError)?;
                }
                Operation::DelayNs(ns) => {
                    crate::sync::KernelDelay.delay_ns(*ns);
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use blueos_hal::err::{HalError, Result as HalResult};
    use blueos_test_macro::test;
    use core::sync::atomic::{AtomicUsize, Ordering};

    /// Mock HAL Spi peripheral for testing BlockSpi
    ///
    /// Implements all required HAL traits (PlatPeri, Configuration, Spi)
    /// to create a BlockSpi wrapper for testing SpiDevice transaction handling.
    static MOCK_SPI: MockHalSpi = MockHalSpi;

    struct MockHalSpi;

    // PlatPeri: required by Spi supertrait (Sync + Send + 'static)
    impl blueos_hal::PlatPeri for MockHalSpi {
        fn enable(&self) {}
        fn disable(&self) {}
    }

    // Configuration<SpiConfig>: required by Spi supertrait
    impl blueos_hal::Configuration<SpiConfig> for MockHalSpi {
        type Target = ();
        fn configure(&self, _param: &SpiConfig) -> HalResult<Self::Target> {
            Ok(())
        }
    }

    // Spi<SpiConfig, ()>: required for BlockSpi::new and SpiDevice impl
    impl blueos_hal::spi::Spi<SpiConfig, ()> for MockHalSpi {
        fn transfer(&self, read: &mut [u8], write: &[u8]) -> HalResult<()> {
            if should_fail.load(Ordering::Relaxed) != 0 {
                return Err(HalError::Fail);
            }
            // Copy write data into read buffer (simulates SPI loopback)
            let len = read.len().min(write.len());
            read[..len].copy_from_slice(&write[..len]);
            // Record the operation
            write_count.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn read(&self, buf: &mut [u8]) -> HalResult<()> {
            if should_fail.load(Ordering::Relaxed) != 0 {
                return Err(HalError::Fail);
            }
            // Fill with configurable test data
            let data = read_data.load(Ordering::Relaxed) as u8;
            for byte in buf.iter_mut() {
                *byte = data;
            }
            read_count.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn write(&self, buf: &[u8]) -> HalResult<()> {
            if should_fail.load(Ordering::Relaxed) != 0 {
                return Err(HalError::Fail);
            }
            // Record the written data
            last_write_len.store(buf.len(), Ordering::Relaxed);
            write_count.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
    }

    // Atomic counters for tracking HAL operations in tests
    static write_count: AtomicUsize = AtomicUsize::new(0);
    static read_count: AtomicUsize = AtomicUsize::new(0);
    static last_write_len: AtomicUsize = AtomicUsize::new(0);
    static read_data: AtomicUsize = AtomicUsize::new(0);
    static should_fail: AtomicUsize = AtomicUsize::new(0);

    fn reset_counters() {
        write_count.store(0, Ordering::Relaxed);
        read_count.store(0, Ordering::Relaxed);
        last_write_len.store(0, Ordering::Relaxed);
        read_data.store(0, Ordering::Relaxed);
        should_fail.store(0, Ordering::Relaxed);
    }

    #[test]
    fn test_block_spi_new() {
        let result = BlockSpi::new(&MOCK_SPI);
        assert!(result.is_ok());
    }

    #[test]
    fn test_block_spi_write_operation() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();

        let write_data = [0x9F, 0x00, 0x00, 0x00];
        block_spi
            .transaction(&mut [Operation::Write(&write_data)])
            .unwrap();

        assert_eq!(write_count.load(Ordering::Relaxed), 1);
        assert_eq!(last_write_len.load(Ordering::Relaxed), 4);
    }

    #[test]
    fn test_block_spi_read_operation() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();
        read_data.store(0xAB, Ordering::Relaxed);

        let mut read_buf = [0u8; 3];
        block_spi
            .transaction(&mut [Operation::Read(&mut read_buf)])
            .unwrap();

        assert_eq!(read_count.load(Ordering::Relaxed), 1);
        assert_eq!(read_buf, [0xAB, 0xAB, 0xAB]);
    }

    #[test]
    fn test_block_spi_transfer_operation() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();

        let mut read_buf = [0u8; 4];
        let write_buf = [0x03, 0x00, 0x10, 0x00];
        block_spi
            .transaction(&mut [Operation::Transfer(&mut read_buf, &write_buf)])
            .unwrap();

        assert_eq!(write_count.load(Ordering::Relaxed), 1);
        // Transfer copies write data into read buffer (loopback behavior)
        assert_eq!(read_buf, write_buf);
    }

    #[test]
    fn test_block_spi_transfer_in_place() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();
        read_data.store(0xFF, Ordering::Relaxed);

        let mut buf = [0xAA, 0xBB, 0xCC, 0xDD];
        block_spi
            .transaction(&mut [Operation::TransferInPlace(&mut buf)])
            .unwrap();

        // TransferInPlace: write then read back (two-step)
        assert_eq!(write_count.load(Ordering::Relaxed), 1);
        assert_eq!(read_count.load(Ordering::Relaxed), 1);
        // After read, buf is filled with mock read data
        assert_eq!(buf, [0xFF, 0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn test_block_spi_mixed_transaction() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();

        let mut read_buf = [0u8; 3];
        read_data.store(0xEF, Ordering::Relaxed);

        block_spi
            .transaction(&mut [Operation::Write(&[0x9F]), Operation::Read(&mut read_buf)])
            .unwrap();

        assert_eq!(write_count.load(Ordering::Relaxed), 1);
        assert_eq!(read_count.load(Ordering::Relaxed), 1);
        assert_eq!(read_buf, [0xEF, 0xEF, 0xEF]);
    }

    #[test]
    fn test_spi_block_error_kind() {
        use embedded_hal::spi::Error as SpiError;

        assert_eq!(
            SpiError::kind(&SpiBlockError::HalError),
            embedded_hal::spi::ErrorKind::Other
        );
        assert_eq!(
            SpiError::kind(&SpiBlockError::InvalidOperation),
            embedded_hal::spi::ErrorKind::Other
        );
    }

    #[test]
    fn test_hal_write_error_propagation() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();
        should_fail.store(1, Ordering::Relaxed);

        let write_data = [0x9F, 0x00, 0x00, 0x00];
        let result = block_spi.transaction(&mut [Operation::Write(&write_data)]);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), SpiBlockError::HalError);
    }

    #[test]
    fn test_hal_read_error_propagation() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();
        should_fail.store(1, Ordering::Relaxed);

        let mut read_buf = [0u8; 3];
        let result = block_spi.transaction(&mut [Operation::Read(&mut read_buf)]);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), SpiBlockError::HalError);
    }

    #[test]
    fn test_hal_transfer_error_propagation() {
        let mut block_spi = BlockSpi::new(&MOCK_SPI).unwrap();
        reset_counters();
        should_fail.store(1, Ordering::Relaxed);

        let mut read_buf = [0u8; 4];
        let write_buf = [0x03, 0x00, 0x10, 0x00];
        let result = block_spi.transaction(&mut [Operation::Transfer(&mut read_buf, &write_buf)]);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), SpiBlockError::HalError);
    }
}
