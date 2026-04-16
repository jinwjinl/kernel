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

pub unsafe fn load_linux_to_guest() {
    use crate::arch::aarch64::virt::guest;
    
    let kernel_dest = guest::LINUX_KERNEL_LOAD_ADDR as *mut u8;
    let dtb_dest = guest::LINUX_DTB_ADDR as *mut u8;

    core::ptr::copy_nonoverlapping(guest::LINUX_IMAGE.as_ptr(), kernel_dest, guest::LINUX_IMAGE.len());
    core::ptr::copy_nonoverlapping(guest::LINUX_DTB.as_ptr(), dtb_dest, guest::LINUX_DTB.len());
    core::arch::asm!("dsb sy", "isb");
}

// Like virt_init but specifically for booting Linux
pub fn virt_boot_linux() {
    // Repeat set in virt_init！！！
    // hyp_init();
    // vgic::init();
    vtimer::init_global_vtimer();
    // mmu_s2::init_stage2(0x4028_0000, 0x0200_0000);

    unsafe { 
        load_linux_to_guest(); 
    }

    unsafe {
        let vcpu = VCPU_MANAGER.0.create_vcpu(0, 0x4028_0000, 0).unwrap();
        vcpu.context_mut().regs[0] = 0x4180_0000;
    }
    vtimer::init_vcpu_timer();


    let result = hvc_call(2, 0, 0);
}