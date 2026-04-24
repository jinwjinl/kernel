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
    unsafe{
        let mut ctlr: u64;
        core::arch::asm!("mrs {}, ICC_CTLR_EL1", out(reg) ctlr);
        if (ctlr & (1 << 1)) == 0{
            // set EOImode.
            ctlr |= 1 << 1;
            core::arch::asm!("msr ICC_CTLR_EL1, {}", in(reg) ctlr);
        }
    }
    let iar: u64;
    unsafe {
        core::arch::asm!("mrs {}, ICC_IAR1_EL1", out(reg) iar);
    }

    let intid = iar & 0xFFFFFF;

    if intid == 1023 {
        // 1023 is Spurious Interrupt of GIC，skip
        return 0;
    }

    if intid != 27 {
        semihosting::println!("[EL2] IRQ trap! INTID: {}", intid);
    }
    if intid == 33 {
        unsafe {
            let lr_val: u64 = (1 << 62) | (1 << 61) | (1 << 60) | (0xA0 << 48) | (33 << 32) | 33;
            
            // Temporarily occupy physical register for uart print in Linux shell
            core::arch::asm!("msr ICH_LR1_EL2, {}", in(reg) lr_val);
            let mut hcr: u64;
            core::arch::asm!("mrs {}, ICH_HCR_EL2", out(reg) hcr);
            hcr |= 1;
            core::arch::asm!("msr ICH_HCR_EL2, {}", in(reg) hcr);
        }
    } else if intid == 27 {
        unsafe {
            let mut ctl: u64;
            core::arch::asm!("mrs {}, CNTV_CTL_EL0", out(reg) ctl);
            ctl |= 1 << 1;
            core::arch::asm!("msr CNTV_CTL_EL0, {}", in(reg) ctl);
        }

        if let Some(vcpu_id) = get_current_vcpu_id() {
                vgic::inject_irq(vcpu_id, 27);
        }

        unsafe{
            core::arch::asm!("msr ICC_EOIR1_EL1, {}", in(reg) iar);
        }
    } else {
        semihosting::println!("[EL2] Unhandled Guest IRQ: {}", intid);
        // For uninterruptible/unknown interrupts, 
        // we must manually downgrade and deactivate them; 
        // otherwise, the interrupt line will be permanently blocked.
        unsafe { 
            core::arch::asm!("msr ICC_EOIR1_EL1, {}", in(reg) iar); 
            core::arch::asm!("msr ICC_DIR_EL1, {}", in(reg) iar);
        }
    }

    unsafe {
        core::arch::asm!("msr ICC_EOIR1_EL1, {}", in(reg) iar);
    }

    if let Some(vcpu_id) = get_current_vcpu_id() {
        vgic::flush(vcpu_id);
    }

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

// pub unsafe fn load_linux_to_guest() {
//     use crate::arch::aarch64::virt::guest;
    
//     let kernel_dest = guest::LINUX_KERNEL_LOAD_ADDR as *mut u8;
//     let dtb_dest = guest::LINUX_DTB_ADDR as *mut u8;

//     core::ptr::copy_nonoverlapping(guest::LINUX_IMAGE.as_ptr(), kernel_dest, guest::LINUX_IMAGE.len());
//     core::ptr::copy_nonoverlapping(guest::LINUX_DTB.as_ptr(), dtb_dest, guest::LINUX_DTB.len());
//     core::arch::asm!("dsb sy", "isb");
// }

// Like virt_init but specifically for booting Linux
pub fn virt_boot_linux() {
    // It will be placed here next.
    // vgic::init();
    vtimer::init_global_vtimer();

    // unsafe { 
    //     load_linux_to_guest(); 
    // }

    unsafe {
        let vcpu = VCPU_MANAGER.0.create_vcpu(0, guest::LINUX_KERNEL_LOAD_ADDR, 0).unwrap();
        vcpu.context_mut().regs[0] = guest::LINUX_DTB_ADDR as u64;
        vcpu.context_mut().spsr = 0x3C5;
    }
    vtimer::init_vcpu_timer();


    let result = hvc_call(2, 0, 0);
}