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

use crate::devices::bus::{BusInterface, BusWrapper};
use blueos_driver::spi::SpiConfig;
use blueos_hal::PlatPeri;
use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    spi::{ErrorType, Operation, SpiBus, SpiDevice},
};
use embedded_hal_bus::spi::DeviceError;

pub struct BlockSpi<T: PlatPeri> {
    inner: &'static T,
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>> BlockSpi<T> {
    pub fn new(inner: &'static T, config: &SpiConfig) -> Result<Self, blueos_hal::err::HalError> {
        inner.configure(config)?;
        Ok(BlockSpi { inner })
    }

    fn read(&mut self, words: &mut [u8]) -> Result<(), crate::error::Error> {
        self.inner.read(words).map_err(|_| crate::error::code::EIO)
    }

    fn write(&mut self, words: &[u8]) -> Result<(), crate::error::Error> {
        self.inner.write(words).map_err(|_| crate::error::code::EIO)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), crate::error::Error> {
        self.inner
            .transfer(read, write)
            .map_err(|_| crate::error::code::EIO)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), crate::error::Error> {
        self.inner
            .write(words)
            .map_err(|_| crate::error::code::EIO)?;
        self.inner.read(words).map_err(|_| crate::error::code::EIO)
    }

    fn flush(&mut self) -> Result<(), crate::error::Error> {
        Ok(())
    }
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>> BusInterface for BlockSpi<T> {
    type Region = ();

    fn read_region(&self, _region: Self::Region, _buffer: &mut [u8]) -> crate::drivers::Result<()> {
        todo!()
    }

    fn write_region(&self, _region: Self::Region, _data: &[u8]) -> crate::drivers::Result<()> {
        todo!()
    }
}

#[cfg(use_embedded_hal_v1)]
impl<T: blueos_hal::spi::Spi<SpiConfig, ()>> ErrorType for BusWrapper<BlockSpi<T>> {
    type Error = crate::error::Error;
}

#[cfg(use_embedded_hal_v1)]
impl embedded_hal::spi::Error for crate::error::Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        // FIXME: Map the error code to embedded_hal::spi::ErrorKind
        embedded_hal::spi::ErrorKind::Other
    }
}

#[cfg(use_embedded_hal_v1)]
impl<T: blueos_hal::spi::Spi<SpiConfig, ()>> SpiBus<u8> for BusWrapper<BlockSpi<T>> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.0.lock().read(words)
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.0.lock().write(words)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.0.lock().transfer(read, write)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.0.lock().transfer_in_place(words)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.lock().flush()
    }
}

pub struct HalOutputPinAdapter<G: blueos_hal::gpio::OutputPin> {
    inner: &'static G,
}

impl<G: blueos_hal::gpio::OutputPin> HalOutputPinAdapter<G> {
    pub const fn new(inner: &'static G) -> Self {
        Self { inner }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HalOutputPinError;

#[cfg(use_embedded_hal_v1)]
impl embedded_hal::digital::Error for HalOutputPinError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

#[cfg(use_embedded_hal_v1)]
impl<G: blueos_hal::gpio::OutputPin> embedded_hal::digital::ErrorType for HalOutputPinAdapter<G> {
    type Error = HalOutputPinError;
}

#[cfg(use_embedded_hal_v1)]
impl<G: blueos_hal::gpio::OutputPin> OutputPin for HalOutputPinAdapter<G> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.inner.set_low().map_err(|_| HalOutputPinError)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.inner.set_high().map_err(|_| HalOutputPinError)
    }
}

pub struct SpinLockDevice<B: BusInterface, CS, D> {
    bus: BusWrapper<B>,
    cs: CS,
    delay: D,
}

impl<B: BusInterface, CS, D> SpinLockDevice<B, CS, D> {
    pub fn new(bus: BusWrapper<B>, mut cs: CS, delay: D) -> Result<Self, CS::Error>
    where
        CS: OutputPin,
    {
        cs.set_high()?;
        Ok(Self { bus, cs, delay })
    }
}

#[cfg(use_embedded_hal_v1)]
impl<B, CS, D> ErrorType for SpinLockDevice<B, CS, D>
where
    B: BusInterface,
    BusWrapper<B>: ErrorType,
    CS: OutputPin,
{
    type Error = DeviceError<<BusWrapper<B> as ErrorType>::Error, CS::Error>;
}

#[cfg(use_embedded_hal_v1)]
impl<Word, B, CS, D> SpiDevice<Word> for SpinLockDevice<B, CS, D>
where
    Word: Copy + 'static,
    B: BusInterface,
    BusWrapper<B>: SpiBus<Word>,
    CS: OutputPin,
    D: DelayNs,
{
    fn transaction(&mut self, operations: &mut [Operation<'_, Word>]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(DeviceError::Cs)?;

        let op_res = operations.iter_mut().try_for_each(|op| match op {
            Operation::Read(buf) => self.bus.read(buf).map_err(DeviceError::Spi),
            Operation::Write(buf) => self.bus.write(buf).map_err(DeviceError::Spi),
            Operation::Transfer(read, write) => {
                self.bus.transfer(read, write).map_err(DeviceError::Spi)
            }
            Operation::TransferInPlace(buf) => {
                self.bus.transfer_in_place(buf).map_err(DeviceError::Spi)
            }
            Operation::DelayNs(ns) => {
                self.bus.flush().map_err(DeviceError::Spi)?;
                self.delay.delay_ns(*ns);
                Ok(())
            }
        });

        let flush_res = self.bus.flush();
        let cs_res = self.cs.set_high();

        op_res?;
        flush_res.map_err(DeviceError::Spi)?;
        cs_res.map_err(DeviceError::Cs)?;

        Ok(())
    }
}
