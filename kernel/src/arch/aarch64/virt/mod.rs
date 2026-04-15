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

pub mod hyp;
pub mod vcpu;
pub mod exit;
pub mod guest;
pub mod mmu_el2;
pub mod mmu_s2;
pub mod vgic;
pub mod vector;
pub mod vtimer;

pub use vcpu::{Vcpu, VcpuManager, VcpuState};
pub use exit::{VmExitReason, VmExitInfo};
pub use hyp::{hyp_init, get_current_el};
pub use vgic::init;
use semihosting::println;

// PL011 UART addresses for QEMU Virt
const UART0_DR: *mut u32 = 0x0900_0000 as *mut u32;
const UART0_FR: *mut u32 = 0x0900_0018 as *mut u32;


#[no_mangle]
pub extern "C" fn trap_irq(_context: &mut crate::arch::aarch64::Context) -> usize {
    semihosting::println!("[EL2] IRQ trap");
    0
}

#[no_mangle]
pub extern "C" fn trap_fiq(_context: &mut crate::arch::aarch64::Context) -> usize {
    semihosting::println!("[EL2] FIQ trap");
    0
}

#[repr(align(16))]
pub struct VcpuManagerWrapper(pub vcpu::VcpuManager);

pub static mut VCPU_MANAGER: VcpuManagerWrapper = 
    VcpuManagerWrapper(vcpu::VcpuManager::new());

#[inline]
pub fn get_current_vcpu_id() -> Option<usize> {
    unsafe { VCPU_MANAGER.0.current_vcpu_id() }
}


pub fn virt_init() {
    hyp_init();
    vgic::init();
}

pub fn hvc_call(func_id: u64, arg1: u64, arg2: u64) -> u64 {
    let result: u64;
    unsafe {
        core::arch::asm!(
            "hvc #0",
            inout("x0") func_id => result,
            in("x1") arg1,
            in("x2") arg2,
            options(nostack)
        );
    }
    result
}