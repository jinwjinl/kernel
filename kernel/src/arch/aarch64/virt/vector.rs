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


use core::arch::asm;
use log::info;
use super::{VCPU_MANAGER, vgic};
use crate::arch::aarch64::virt::exit::handle_vm_exit;

static mut PRINTED_ALIGN: bool = false;

const VECTOR_TABLE_SIZE: usize = 2048;
const SYNC_EXCEPTION_OFFSET: usize = 0x400;

#[inline]
pub fn get_vector_table_addr() -> usize {
    hypervisor_vectors as *const () as usize
}

#[naked]
#[no_mangle]
#[link_section = ".text.hypervisor_vectors"]
pub unsafe extern "C" fn hypervisor_vectors() {
    core::arch::naked_asm!(
        "b sync_current_sp0\n",
        ".space 124\n",
        "b irq_current\n",
        ".space 124\n",
        "b fiq_current\n",
        ".space 124\n",
        "b serror_current\n",
        ".space 124\n",
        "b sync_current_spx\n",
        ".space 124\n",
        "b irq_current\n",
        ".space 124\n",
        "b fiq_current\n",
        ".space 124\n",
        "b serror_current\n",
        ".space 124\n",
        "b sync_from_lower_el1\n",
        ".space 124\n",
        "b irq_from_lower_el1\n",
        ".space 124\n",
        "b fiq_from_lower_el1\n",
        ".space 124\n",
        "b serror_from_lower_el1\n",
        ".space 124\n",
        "b sync_current_spx\n",
        ".space 124\n",
        "b irq_current\n",
        ".space 124\n",
        "b fiq_current\n",
        ".space 124\n",
        "b serror_current\n",
        ".space 124\n"
    );
}

#[naked]
#[no_mangle]
pub unsafe extern "C" fn sync_from_lower_el1() {
    core::arch::naked_asm!(
        "sub sp, sp, #272\n",
        "stp x0, x1, [sp, #0]\n",
        "stp x2, x3, [sp, #16]\n",
        "stp x4, x5, [sp, #32]\n",
        "stp x6, x7, [sp, #48]\n",
        "stp x8, x9, [sp, #64]\n",
        "stp x10, x11, [sp, #80]\n",
        "stp x12, x13, [sp, #96]\n",
        "stp x14, x15, [sp, #112]\n",
        "stp x16, x17, [sp, #128]\n",
        "stp x18, x19, [sp, #144]\n",
        "stp x20, x21, [sp, #160]\n",
        "stp x22, x23, [sp, #176]\n",
        "stp x24, x25, [sp, #192]\n",
        "stp x26, x27, [sp, #208]\n",
        "stp x28, x29, [sp, #224]\n",
        "str x30, [sp, #240]\n",
        "mrs x1, elr_el2\n",
        "mrs x2, spsr_el2\n",
        "str x1, [sp, #248]\n",
        "str x2, [sp, #256]\n",
        "mov x0, sp\n",
        "bl sync_from_lower_el1_rust\n",
        "cbz x0, 1f\n",
        "ldr x1, [sp, #248]\n",
        "ldr x2, [sp, #256]\n",
        "msr elr_el2, x1\n",
        "msr spsr_el2, x2\n",
        "ldp x0, x1, [sp, #0]\n",
        "ldp x2, x3, [sp, #16]\n",
        "ldp x4, x5, [sp, #32]\n",
        "ldp x6, x7, [sp, #48]\n",
        "ldp x8, x9, [sp, #64]\n",
        "ldp x10, x11, [sp, #80]\n",
        "ldp x12, x13, [sp, #96]\n",
        "ldp x14, x15, [sp, #112]\n",
        "ldp x16, x17, [sp, #128]\n",
        "ldp x18, x19, [sp, #144]\n",
        "ldp x20, x21, [sp, #160]\n",
        "ldp x22, x23, [sp, #176]\n",
        "ldp x24, x25, [sp, #192]\n",
        "ldp x26, x27, [sp, #208]\n",
        "ldp x28, x29, [sp, #224]\n",
        "ldr x30, [sp, #240]\n",
        "add sp, sp, #272\n",
        "eret\n",
        "1:\n",
        "wfi\n",
        "b 1b\n"
    );
}

#[no_mangle]
pub unsafe extern "C" fn sync_from_lower_el1_rust(frame: *mut u64) -> u64 {
    if let Some(id) = VCPU_MANAGER.0.current_vcpu_id() {
        if let Some(vcpu) = VCPU_MANAGER.0.get_vcpu(id) {
            {
                let context = vcpu.context_mut();

                for i in 0..31 {
                    context.regs[i] = *frame.add(i);
                }
                context.elr_el2 = *frame.add(31);
                context.spsr = *frame.add(32);
            }

            if !PRINTED_ALIGN {
                let addr = core::ptr::addr_of!(VCPU_MANAGER) as usize;
                info!("[ALIGN] VCPU_MANAGER addr={:#018x} mod16={}", addr, addr & 0xF);
                PRINTED_ALIGN = true;
            }

            let ok = handle_vm_exit(vcpu);
            let original_elr = *frame.add(31);
            let context = vcpu.context();
            
            // Always use the context's ELR, which should have been updated by the handler
            let return_elr = context.elr_el2;
            
            // Sanity check: Guest should not jump to Host memory
            if (0x4000_0000..0x4800_0000).contains(&return_elr) {
                info!("[VECTORS] CRITICAL: Guest attempting to jump to Host memory: {:#x}", return_elr);
                info!("[VECTORS] This indicates memory corruption or stack smash.");
            }

            // DEBUG: Check for NULL return address
            if return_elr == 0 {
                info!("[VECTORS] CRITICAL: ELR is 0! We are about to crash.");
                info!("[VECTORS] Original ELR from stack: {:#x}", original_elr);
                info!("[VECTORS] Context ELR: {:#x}", context.elr_el2);
            }
            
            if return_elr == original_elr {
                info!("[VECTORS] Warning: ELR unchanged after exit! PC={:#x}", return_elr);
            } else {
                info!("[VECTORS] ELR updated: {:#x} -> {:#x}", original_elr, return_elr);
            }

            *frame.add(31) = return_elr;
            *frame.add(0) = context.regs[0];
            // *frame.add(33) = context.vbar_el1; // FIXME: VBAR not saved in asm
            
            // Flush VGIC LRs before re-entering Guest
            vgic::flush(id);

            return if ok { 1 } else { 0 };
        }
    }

    0
}

const HCR_EL2_VI: u64 = 1 << 7;
const HCR_EL2_VF: u64 = 1 << 6;

/// Solve irq from lower el1.
#[naked]
#[no_mangle]
pub unsafe extern "C" fn irq_from_lower_el1() {
    core::arch::naked_asm!(
        "sub sp, sp, #272\n",
        "stp x0, x1, [sp, #0]\n",
        "stp x2, x3, [sp, #16]\n",
        "stp x4, x5, [sp, #32]\n",
        "stp x6, x7, [sp, #48]\n",
        "stp x8, x9, [sp, #64]\n",
        "stp x10, x11, [sp, #80]\n",
        "stp x12, x13, [sp, #96]\n",
        "stp x14, x15, [sp, #112]\n",
        "stp x16, x17, [sp, #128]\n",
        "stp x18, x19, [sp, #144]\n",
        "stp x20, x21, [sp, #160]\n",
        "stp x22, x23, [sp, #176]\n",
        "stp x24, x25, [sp, #192]\n",
        "stp x26, x27, [sp, #208]\n",
        "stp x28, x29, [sp, #224]\n",
        "str x30, [sp, #240]\n",
        "mrs x1, elr_el2\n",
        "mrs x2, spsr_el2\n",
        "str x1, [sp, #248]\n",
        "str x2, [sp, #256]\n",
        "mov x0, sp\n",
        "bl trap_irq\n",
        "ldr x1, [sp, #248]\n",
        "ldr x2, [sp, #256]\n",
        "msr elr_el2, x1\n",
        "msr spsr_el2, x2\n",
        "ldp x0, x1, [sp, #0]\n",
        "ldp x2, x3, [sp, #16]\n",
        "ldp x4, x5, [sp, #32]\n",
        "ldp x6, x7, [sp, #48]\n",
        "ldp x8, x9, [sp, #64]\n",
        "ldp x10, x11, [sp, #80]\n",
        "ldp x12, x13, [sp, #96]\n",
        "ldp x14, x15, [sp, #112]\n",
        "ldp x16, x17, [sp, #128]\n",
        "ldp x18, x19, [sp, #144]\n",
        "ldp x20, x21, [sp, #160]\n",
        "ldp x22, x23, [sp, #176]\n",
        "ldp x24, x25, [sp, #192]\n",
        "ldp x26, x27, [sp, #208]\n",
        "ldp x28, x29, [sp, #224]\n",
        "ldr x30, [sp, #240]\n",
        "add sp, sp, #272\n",
        "eret\n",
    );
}

/// Solve fiq from lower el1.
#[naked]
#[no_mangle]
pub unsafe extern "C" fn fiq_from_lower_el1() {
     core::arch::naked_asm!(
        "sub sp, sp, #272\n",
        "stp x0, x1, [sp, #0]\n",
        "stp x2, x3, [sp, #16]\n",
        "stp x4, x5, [sp, #32]\n",
        "stp x6, x7, [sp, #48]\n",
        "stp x8, x9, [sp, #64]\n",
        "stp x10, x11, [sp, #80]\n",
        "stp x12, x13, [sp, #96]\n",
        "stp x14, x15, [sp, #112]\n",
        "stp x16, x17, [sp, #128]\n",
        "stp x18, x19, [sp, #144]\n",
        "stp x20, x21, [sp, #160]\n",
        "stp x22, x23, [sp, #176]\n",
        "stp x24, x25, [sp, #192]\n",
        "stp x26, x27, [sp, #208]\n",
        "stp x28, x29, [sp, #224]\n",
        "str x30, [sp, #240]\n",
        "mrs x1, elr_el2\n",
        "mrs x2, spsr_el2\n",
        "str x1, [sp, #248]\n",
        "str x2, [sp, #256]\n",
        "mov x0, sp\n",
        "bl trap_fiq\n",
        "ldr x1, [sp, #248]\n",
        "msr elr_el2, x1\n",
        "ldp x0, x1, [sp, #0]\n",
        "ldp x2, x3, [sp, #16]\n",
        "ldp x4, x5, [sp, #32]\n",
        "ldp x6, x7, [sp, #48]\n",
        "ldp x8, x9, [sp, #64]\n",
        "ldp x10, x11, [sp, #80]\n",
        "ldp x12, x13, [sp, #96]\n",
        "ldp x14, x15, [sp, #112]\n",
        "ldp x16, x17, [sp, #128]\n",
        "ldp x18, x19, [sp, #144]\n",
        "ldp x20, x21, [sp, #160]\n",
        "ldp x22, x23, [sp, #176]\n",
        "ldp x24, x25, [sp, #192]\n",
        "ldp x26, x27, [sp, #208]\n",
        "ldp x28, x29, [sp, #224]\n",
        "ldr x30, [sp, #240]\n",
        "add sp, sp, #272\n",
        "eret\n",
    );
}

/// Solve serror from lower el1.
#[no_mangle]
pub unsafe extern "C" fn serror_from_lower_el1() {
    info!("[VECTOR] is from EL1 SError!");
    asm!("eret", options(noreturn));
}

/// Solve sync exception from lower el2 sp0.
#[no_mangle]
pub unsafe extern "C" fn sync_current_sp0() {
    info!("[VECTOR] is from EL2 SP0 sync exception!");
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn sync_current_spx() {
    let esr: u64;
    let elr: u64;
    let far: u64;
    asm!("mrs {}, esr_el2", out(reg) esr, options(nostack));
    asm!("mrs {}, elr_el2", out(reg) elr, options(nostack));
    asm!("mrs {}, far_el2", out(reg) far, options(nostack));

    info!("[VECTOR] is from EL2 SPx sync exception!");
    info!("ESR_EL2: {:#x}", esr);
    info!("ELR_EL2: {:#x}", elr);
    info!("FAR_EL2: {:#x}", far);

    let ec = (esr >> 26) & 0x3F;
    let iss = esr & 0x1FFFFFF;
    info!("EC: {:#x}, ISS: {:#x}", ec, iss);

    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn sync_current_el1() {
    let esr: u64;
    let elr: u64;
    let far: u64;
    let spsr: u64;
    asm!("mrs {}, esr_el2", out(reg) esr, options(nostack));
    asm!("mrs {}, elr_el2", out(reg) elr, options(nostack));
    asm!("mrs {}, far_el2", out(reg) far, options(nostack));
    asm!("mrs {}, spsr_el2", out(reg) spsr, options(nostack));
    info!("[VECTOR] is from EL2 EL1 sync exception!");
    info!("[VECTOR]   ESR_EL2={:#x} ELR_EL2={:#x} FAR_EL2={:#x} SPSR_EL2={:#x}", esr, elr, far, spsr);
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn sync_current_el0() {
    info!("[VECTOR] is from EL2 EL0 sync exception!");
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn irq_current() {
    info!("[VECTOR] is from EL2 IRQ!");
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn fiq_current() {
    info!("[VECTOR] is from EL2 FIQ!");
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn serror_current() {
    info!("[VECTOR] is from EL2 SError!");
    loop { asm!("wfi"); }
}