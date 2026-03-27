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
use super::{VCPU_MANAGER, early_uart_print, early_uart_print_hex, vgic, hyp, guest};
use crate::arch::aarch64::virt::exit::{handle_vm_exit, is_guest_shutdown, clear_guest_shutdown};

static mut PRINTED_ALIGN: bool = false;

const VECTOR_TABLE_SIZE: usize = 2048;
const SYNC_EXCEPTION_OFFSET: usize = 0x400;

core::arch::global_asm!(
    "
.section .text.hyper_vector_table
.align 11
.global hyper_vector_table
hyper_vector_table:
    // Current EL with SP0
    .align 7
        b sync_current_sp0
    .align 7
        b irq_current
    .align 7
        b fiq_current
    .align 7
        b serror_current

    // Current EL with SPx
    .align 7
        b sync_current_spx
    .align 7
        b irq_current
    .align 7
        b fiq_current
    .align 7
        b serror_current

    // Lower EL using AArch64
    .align 7
        b sync_from_lower_el1
    .align 7
        b irq_from_lower_el1
    .align 7
        b fiq_from_lower_el1
    .align 7
        b serror_from_lower_el1

    // Lower EL using AArch32
    .align 7
        b sync_current_spx   // Should not happen for now
    .align 7
        b irq_current
    .align 7
        b fiq_current
    .align 7
        b serror_current
"
);

extern "C" {
    fn hyper_vector_table();
}

#[inline]
pub fn get_vector_table_addr() -> usize {
    hyper_vector_table as *const () as usize
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
        "mrs x3, sp_el1\n",
        "str x1, [sp, #248]\n",
        "str x2, [sp, #256]\n",
        "str x3, [sp, #264]\n",
        "mov x0, sp\n",
        "bl sync_from_lower_el1_rust\n",
        // x0 == 0, Guest dump.
        "cbz x0, 3f\n",
        // x0 == 2, Guest shutdown, return to Host.
        "cmp x0, #2\n",
        "b.eq 2f\n",
        // x0 == 1, continue running Guest.
        "ldr x1, [sp, #248]\n",
        "ldr x2, [sp, #256]\n",
        "ldr x3, [sp, #264]\n",
        "msr elr_el2, x1\n",
        "msr spsr_el2, x2\n",
        "msr sp_el1, x3\n",
        "isb\n",
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
        "2:\n",
        "ldr x1, [sp, #248]\n",   // Host ELR
        "ldr x2, [sp, #256]\n",   // Host SPSR
        "ldr x3, [sp, #264]\n",   // Host SP_EL1
        "msr elr_el2, x1\n",
        "msr spsr_el2, x2\n",
        "msr sp_el1, x3\n",
        "isb\n",
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
        "3:\n",
        "wfi\n",
        "b 3b\n"
    );
}

#[no_mangle]
pub unsafe extern "C" fn sync_from_lower_el1_rust(frame: *mut u64) -> u64 {
    // Hvc from guest.
    if let Some(id) = VCPU_MANAGER.0.current_vcpu_id() {
        if let Some(vcpu) = VCPU_MANAGER.0.get_vcpu(id) {
            {
                let context = vcpu.context_mut();
                for i in 0..31 {
                    context.regs[i] = *frame.add(i);
                }
                context.elr_el2 = *frame.add(31);
                context.spsr = *frame.add(32);
                context.sp = *frame.add(33);
            }

            let ok = handle_vm_exit(vcpu);
            if !ok && is_guest_shutdown() {
                clear_guest_shutdown();
                VCPU_MANAGER.0.clear_current_vcpu();
                *frame.add(31) = VCPU_MANAGER.0.host_elr;
                *frame.add(32) = VCPU_MANAGER.0.host_spsr;
                *frame.add(33) = VCPU_MANAGER.0.host_sp;
                
                // Restore Host GPRs (x0-x30)
                for i in 0..31 {
                    *frame.add(i) = VCPU_MANAGER.0.host_regs[i];
                }
                let host_vbar = VCPU_MANAGER.0.host_vbar;
                    core::arch::asm!(
                    "msr vbar_el1, {v}",
                    "isb",
                    v = in(reg) host_vbar,
                    options(nostack, nomem)
                );
                
                core::arch::asm!("dsb sy", options(nostack, nomem));

                // unsafe { 
                //     early_uart_print_hex("[SHUTDOWN] Writing host_elr to frame", VCPU_MANAGER.0.host_elr);
                //     early_uart_print_hex("[SHUTDOWN] Writing host_spsr to frame", VCPU_MANAGER.0.host_spsr);
                //     early_uart_print_hex("[SHUTDOWN] Writing host_sp to frame", VCPU_MANAGER.0.host_sp);
                //     early_uart_print_hex("[SHUTDOWN] frame ptr", frame as u64);
                // }
                
                return 2;
            }

            let context = vcpu.context();
            
            *frame.add(31) = context.elr_el2;
            *frame.add(32) = context.spsr;
            *frame.add(33) = context.sp;
            for i in 0..31 {
            *frame.add(i) = context.regs[i];
        }
            
            vgic::flush(id);
            return if ok { 1 } else { 0 };
        }
    }

    // Hvc from host.
    let esr: u64;
    let elr: u64;
    asm!("mrs {}, esr_el2", out(reg) esr, options(nostack));
    asm!("mrs {}, elr_el2", out(reg) elr, options(nostack));
    let ec = (esr >> 26) & 0x3F;

    // EC = 0x16 (HVC64)
    if ec == 0x16 {
        // Use x0 (func_id) for dispatch instead of ISS
        let func_id = *frame.add(0);
        
        match func_id {
            0 => { 
                early_uart_print("[EL2] VMM_INIT: Host requested init (already done in boot).");
                core::ptr::write_volatile(frame.add(0), 0u64); // Success
                early_uart_print_hex("[EL2] Return ELR", *frame.add(31));
            }
            0x01 => { // HVC #1: VCPU_INIT (Create VCPU)
                early_uart_print("[EL2] VCPU_INIT: Creating VCPU 0...");
                
                // Run guest in-place. Stage-2 MMU will be configured to identity-map this address.
                let entry = guest::guest_entry as usize;
                let stack_top = guest::GUEST_STACK_TOP;
                
                early_uart_print_hex("[EL2] VCPU_INIT: entry=", entry as u64);
                match VCPU_MANAGER.0.create_vcpu(0, entry, stack_top) {
                    Ok(_) => {
                        early_uart_print("[EL2] VCPU_INIT: OK, setting x0=0");
                        core::ptr::write_volatile(frame.add(0), 0u64);
                    },
                    Err(e) => {
                        early_uart_print("[EL2] VCPU_INIT: FAILED");
                        core::ptr::write_volatile(frame.add(0), 1u64);
                    }
                }
            }
            0x02 => { // HVC #2: VCPU_RUN (Run VCPU)
                early_uart_print("[EL2] VCPU_RUN: Switching to Guest...");
                // Save host return address and state
                VCPU_MANAGER.0.host_elr  = *frame.add(31);
                VCPU_MANAGER.0.host_spsr = *frame.add(32);
                VCPU_MANAGER.0.host_sp   = *frame.add(33);
                
                // Save Host GPRs (x0-x30)
                for i in 0..31 {
                    VCPU_MANAGER.0.host_regs[i] = *frame.add(i);
                }

                let vbar: u64;
                core::arch::asm!("mrs {}, vbar_el1",out(reg) vbar);
                VCPU_MANAGER.0.host_vbar = vbar;
                early_uart_print_hex("[EL2] Host ELR saved", VCPU_MANAGER.0.host_elr);
                early_uart_print_hex("[EL2] Host SPSR saved", VCPU_MANAGER.0.host_spsr);
                early_uart_print_hex("[EL2] Host SP saved", VCPU_MANAGER.0.host_sp);
                
                // CRITICAL: Enable Virtualization (VM bit) ONLY before entering Guest
                hyp::configure_hcr_el2_for_guest();  
                VCPU_MANAGER.0.run_vcpu(0);
            }
            _ => {
                early_uart_print_hex("[EL2] Unknown Host HVC: ", func_id);
                early_uart_print_hex("[EL2]   ELR", elr);
            }
        }        
        return 1; // Resume
    }

    // 2. Handle FP/SIMD Trap (Allow Host to use FP)
    // EC = 0x07 (Access to SIMD/FP)
    if ec == 0x07 {
        asm!("msr cptr_el2, xzr");
        return 1; 
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
        "mov x19, sp\n",
        "bl trap_irq\n",
        "mov sp, x19\n",
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
    // kprintln!("[VECTOR] is from EL1 SError!");
    asm!("eret", options(noreturn));
}

/// Solve sync exception from lower el2 sp0.
#[no_mangle]
pub unsafe extern "C" fn sync_current_sp0() {
    // kprintln!("[VECTOR] is from EL2 SP0 sync exception!");
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

    early_uart_print("[EL2] CRITICAL: Sync Exception from EL2 (SPx)!");
    early_uart_print_hex("  ESR_EL2", esr);
    early_uart_print_hex("  ELR_EL2", elr);
    early_uart_print_hex("  FAR_EL2", far);
    
    // Attempt to decode syndrome
    let ec = (esr >> 26) & 0x3F;
    early_uart_print_hex("  Exception Class (EC)", ec);
    
    // Data Abort from same EL
    if ec == 0x25 {
        early_uart_print("[EL2] Data Abort (EL2)");
    }

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
    // kprintln!("[VECTOR] is from EL2 EL1 sync exception!");
    // kprintln!("[VECTOR]   ESR_EL2={:#x} ELR_EL2={:#x} FAR_EL2={:#x} SPSR_EL2={:#x}", esr, elr, far, spsr);
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn sync_current_el0() {
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn irq_current() {
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn fiq_current() {
    loop { asm!("wfi"); }
}

#[no_mangle]
pub unsafe extern "C" fn serror_current() {
    loop { asm!("wfi"); }
}