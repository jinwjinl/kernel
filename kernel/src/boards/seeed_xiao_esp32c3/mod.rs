// Copyright (c) 2025 vivo Mobile Communication Co., Ltd.
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

mod config;
use crate::{
    arch,
    arch::riscv::{local_irq_enabled, trap_entry, Context},
    scheduler, time,
};
use blueos_driver::{interrupt_controller::Interrupt, uart::esp32_usb_serial::Esp32UsbSerialIsr};
use blueos_hal::{isr::IsrDesc, Has8bitDataReg};

// FIXME: Only support unit0 for now
pub type ClockImpl =
    blueos_driver::systimer::esp32_sys_timer::Esp32SysTimer<0x6002_3000, 16_000_000>;

core::arch::global_asm!(
    "
.section .trap
.type _vector_table, @function

.option push
.balign 0x4
.option norelax
.option norvc

_vector_table:
    j {trap_entry}          // 0: Exception 
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    ",
    trap_entry = sym trap_entry,
);

#[inline]
fn init_vector_table() {
    unsafe extern "C" {
        static _vector_table: u32;
    }
    let mut v = core::ptr::addr_of!(_vector_table) as usize;
    v |= 1; // set the least significant bit to enable vectored mode
    unsafe {
        core::arch::asm!(
            "csrw mtvec, {0}",
            in(reg) v,
            options(nostack, preserves_flags),
        );
    }
}

pub(crate) fn handle_intc_irq(ctx: &Context, mcause: usize, mtval: usize) {
    let cpu_id = arch::current_cpu_id();
    match mcause & 0xff {
        TARGET0_INT_NUM => {
            ClockImpl::clear_interrupt();
            crate::time::handle_clock_interrupt();
        }
        USB_SERIAL_JTAG_INT_NUM => {
            ESP32_USB_SERIAL_ISR.service_isr();
        }
        _ => {}
    }
}

const TARGET0_INT_NUM: usize = 16;

const USB_SERIAL_JTAG_INT_NUM: usize = 15;

const RTC_CNTL_BASE: usize = 0x6000_8000;
const RTC_CNTL_WDTWRITECT_REG: usize = RTC_CNTL_BASE + 0xA8;
const RTC_CNTL_WDTCONFIG0_REG: usize = RTC_CNTL_BASE + 0x90;

const USB_SERIAL_JTAG_IRQ: Interrupt = Interrupt::new(26, USB_SERIAL_JTAG_INT_NUM);
const SYSTIMER_TARGET0_IRQ: Interrupt = Interrupt::new(37, TARGET0_INT_NUM);

pub(crate) fn init() {
    assert!(!local_irq_enabled());

    crate::boot::init_runtime();
    crate::boot::init_heap();
    init_vector_table();

    blueos_driver::systimer::esp32_sys_timer::Esp32SysTimer::<0x6002_3000, 16_000_000>::init();

    unsafe {
        // disable WDT to avoid unexpected reset
        core::ptr::write_volatile(RTC_CNTL_WDTWRITECT_REG as *mut u32, 0x50D83AA1);
        core::ptr::write_volatile(RTC_CNTL_WDTCONFIG0_REG as *mut u32, 0);
        core::ptr::write_volatile(RTC_CNTL_WDTWRITECT_REG as *mut u32, 0);
    }

    get_device!(intc).allocate_irq(SYSTIMER_TARGET0_IRQ);
    get_device!(intc).allocate_irq(USB_SERIAL_JTAG_IRQ);

    get_device!(intc).set_threshold(1);

    get_device!(intc).set_priority(USB_SERIAL_JTAG_IRQ, 15);
    get_device!(intc).set_priority(SYSTIMER_TARGET0_IRQ, 15);
    get_device!(intc).enable_irq(SYSTIMER_TARGET0_IRQ);
    get_device!(intc).enable_irq(USB_SERIAL_JTAG_IRQ);
}

crate::define_peripheral! {
    (console_uart, blueos_driver::uart::esp32_usb_serial::Esp32UsbSerial,
     blueos_driver::uart::esp32_usb_serial::Esp32UsbSerial::new()),
    (intc, blueos_driver::interrupt_controller::esp32_intc::Esp32Intc,
     blueos_driver::interrupt_controller::esp32_intc::Esp32Intc::new(0x600c_2000)),
    (spi2, blueos_driver::spi::esp32_spi2::Esp32Spi2,
     blueos_driver::spi::esp32_spi2::Esp32Spi2::new()),
}

// Pin states are configured directly in init_block_devices() using Esp32IoMuxPinctrl

#[cfg(fatfs)]
pub const BLOCK_STORAGE_DEVICE_NAME: &str = "flash-storage";
#[cfg(fatfs)]
pub const BLOCK_STORAGE_MOUNT_POINT: &str = "data";

#[cfg(enable_block)]
pub(crate) fn init_block_devices() {
    use crate::devices::{spi_core::block_spi::BlockSpiBus, storage::spi_flash};
    use blueos_driver::gpio::esp32_gpio::{Esp32GpioOutputPin, Esp32IoMuxPinctrl};
    use blueos_hal::pinctrl::AlterFuncPin;
    use embedded_hal_bus::spi::ExclusiveDevice;

    // Configure SPI2 pins via IO_MUX + GPIO Matrix
    // Seeed XIAO ESP32-C3: SCK=GPIO8, MISO=GPIO9, MOSI=GPIO10, CS=GPIO5
    const PIN_STATES: [Esp32IoMuxPinctrl; 4] = [
        // SCK (GPIO8) — SPI2 clock output via GPIO Matrix (FSPICLK_OUT_IDX=63)
        Esp32IoMuxPinctrl::new(8, 1, false, false, false, 2, Some(63), None, false),
        // MISO (GPIO9) — SPI2 data input via GPIO Matrix (FSPIQ_IN_IDX=64)
        Esp32IoMuxPinctrl::new(9, 1, true, false, false, 2, None, Some(64), false),
        // MOSI (GPIO10) — SPI2 data output via GPIO Matrix (FSPID_OUT_IDX=65)
        Esp32IoMuxPinctrl::new(10, 1, false, false, false, 2, Some(65), None, false),
        // CS (GPIO5) — software-controlled via GPIO output (FSPICS0_OUT_IDX=68)
        Esp32IoMuxPinctrl::new(5, 1, false, true, false, 2, Some(68), None, true),
    ];
    for pin in PIN_STATES {
        pin.init();
    }

    let spi2 = get_device!(spi2);
    let bus = BlockSpiBus::new(spi2).expect("Failed to configure SPI2 for flash");
    let cs = Esp32GpioOutputPin::<5>::new();
    let spi_dev = ExclusiveDevice::new(bus, cs, crate::sync::KernelDelay)
        .expect("Failed to create SPI flash device");
    spi_flash::init_spi_flash(spi_dev).expect("SPI flash initialization failed");
}

#[inline(always)]
pub(crate) fn send_ipi(_hart: usize) {}

#[inline(always)]
pub(crate) fn clear_ipi(_hart: usize) {}

static ESP32_USB_SERIAL_ISR: Esp32UsbSerialIsr<0x6004_3000, crate::drivers::serial::Serial> =
    Esp32UsbSerialIsr::<0x6004_3000, _> {
        data: &crate::drivers::serial::TTY_SERIAL,
        tx_isr: Some(crate::drivers::serial::Serial::xmitchars),
        rx_isr: Some(crate::drivers::serial::Serial::recvchars),
    };
