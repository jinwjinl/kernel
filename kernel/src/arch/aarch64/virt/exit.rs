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
use super::{vcpu::Vcpu, vgic, hyp};
use semihosting::println;

static mut GUEST_SHUTDOWN: bool = false;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VmExitReason {
    Hvc,
    Svc,
    DataAbortLowerEL,
    InstructionAbortLowerEL,
    TrappedWfiWfe,
    Unknown(u32),
}

#[derive(Debug)]
pub struct VmExitInfo {
    pub reason: VmExitReason,
    pub esr: u64,
    pub far: usize,
    pub pstate: u64,
    pub return_addr: usize,
}

#[inline]
pub fn parse_exit_reason(esr: u64) -> VmExitReason {
    let ec = (esr >> 26) & 0x3F;
    
    match ec {
        0x16 => VmExitReason::Hvc,
        0x15 => VmExitReason::Svc,  
        0x24 => VmExitReason::DataAbortLowerEL,
        0x20 => VmExitReason::InstructionAbortLowerEL,
        0x01 => VmExitReason::TrappedWfiWfe,
        _ => VmExitReason::Unknown(ec as u32),
    }
}

pub fn handle_vm_exit(vcpu: &mut Vcpu) -> bool {
    vgic::sync(vcpu.id());
    let _context = vcpu.context_mut();
    let esr = read_esr_el2();
    let elr = read_elr_el2();
    let pstate = read_spsr_el2();
    let reason = parse_exit_reason(esr);

    let exit_info = VmExitInfo {
        reason,
        esr,
        far: elr,
        pstate,
        return_addr: elr + 4,  
    };
    
    semihosting::println!("[EXIT] VM Exit Happened!");
    semihosting::println!("[EXIT]  Reason: {:?}", reason);
    semihosting::println!("[EXIT]   ESR: {:#x}", esr);
    semihosting::println!("[EXIT]   EC: {:#x}", (esr >> 26) & 0x3F);
    semihosting::println!("[EXIT]   FAR: {:#x}", elr);
    semihosting::println!("[EXIT]   PSTATE: {:#x}", pstate);
    
    match reason {
        VmExitReason::Hvc => {
            handle_hvc(vcpu, &exit_info)
        }
        VmExitReason::Svc => {
            handle_svc(vcpu, &exit_info)
        }
        VmExitReason::DataAbortLowerEL => {
            semihosting::println!("[EXIT] Data Abort from Guest (Stage-2 Fault)");
            let iss = esr & 0x1FFFFFF;
            let dfsc = iss & 0x3F;
            let is_write = (iss & (1 << 6)) != 0;
            let faulting_pc = vcpu.context().elr_el2; 

            unsafe { 
                semihosting::println!("=====================================");
                semihosting::println!("[EXIT] PoC Guest triggered Data Abort!");
                semihosting::println!("[EXIT]   1. Faulting PC (ELR_EL2) : {}", faulting_pc);
                semihosting::println!("[EXIT]   2. Target Addr (FAR_EL2) : {}", {
                let far: u64;
                core::arch::asm!("mrs {}, far_el2", out(reg) far, options(nostack));
                far
            });
                if is_write {
                    semihosting::println!("[EXIT]   3. Access Type           : WRITE");
                } else {
                    semihosting::println!("[EXIT]   3. Access Type           : READ");
                }
                semihosting::println!("[EXIT]   4. DFSC Code             : {}", dfsc as u64);
                semihosting::println!("=====================================");
            }

            if (dfsc & 0x3C) == 0x04 || (dfsc & 0x3C) == 0x08 || (dfsc & 0x3C) == 0x0C {
                // Translation fault (level 0/1/2/3) - Stage-2 未映射
                unsafe { 
                    semihosting::println!("[EXIT]   Stage-2 Translation Fault - skipping instruction"); 
                    let far: u64;
                    core::arch::asm!("mrs {}, far_el2", out(reg) far, options(nostack));
                    let handled = vgic::handle_data_abort(
                            vcpu.id(),
                            esr,
                            far as u64,
                            &mut vcpu.context_mut().regs
                        );
                
                        if handled {
                            semihosting::println!("[EXIT]   MMIO Handled by vGIC");
                            vcpu.context_mut().elr_el2 += 4;
                            vgic::flush(vcpu.id());
                            return true;
                        } else {
                            semihosting::println!("[EXIT]   Unhandled Stage-2 Address!");
                        }
                }
            }
            semihosting::println!("[EXIT]   Unrecoverable Data Abort, terminating Guest");
            false
        }
        VmExitReason::InstructionAbortLowerEL => {
            semihosting::println!("[EXIT] Instruction Abort from Guest!");
            let iss = esr & 0x1FFFFFF;
            let ifsc = iss & 0x3F;
            
            if (ifsc & 0x3C) == 0x14 {
                 semihosting::println!("[EXIT]   Stage-2 Translation Fault (Instruction)!");
            }
             false
        }
        VmExitReason::TrappedWfiWfe => {
           semihosting::println!("[EXIT] Trapped WFI/WFE instruction");
            vcpu.context_mut().elr_el2 += 4;
            true
        }
        VmExitReason::Unknown(ec) => {
            semihosting::println!("[EXIT]  Unknown Exit Reason: EC = {:#x}", ec);
            false
        }
    }
}

fn handle_hvc(vcpu: &mut Vcpu, info: &VmExitInfo) -> bool {
    let saved_x0 = vcpu.context().regs[0];
    semihosting::println!("[EXIT] Handle HVC Call");
    let vcpu_id = vcpu.id();
    let context = vcpu.context_mut();
    let hvc_num = info.esr & 0xFFFF;
    semihosting::println!("[EXIT]   HVC#{}", hvc_num);
    
    // Easy HVC Services.
    match hvc_num {
        0x00 => {
            let psci_func_id = context.regs[0];
            if psci_func_id == 0x84000008 { // PSCI_SYSTEM_OFF
                unsafe { 
                    semihosting::println!("[EXIT] HVC#0: Linux requested PSCI_SYSTEM_OFF. Shutting down..."); 
                    GUEST_SHUTDOWN = true;
                }
                #[cfg(not(test))]
                super::hyp::shutdown_guest();
                return false;
            } else {
                semihosting::println!("[EXIT] Ignored PSCI call: {}", psci_func_id);
                context.elr_el2 += 4;
                return true;
            }
        }
        0x01 => {
            semihosting::println!("[EXIT]   ESR_EL1: {}", context.regs[0]);
            context.elr_el2 += 4;
            return true;
        }
        0x10 => {
            semihosting::println!("[EXIT] HVC#0x10: Guest status, x0= {}", context.regs[0]);
            match context.regs[0] {
                0xDEAD_BEEF => { semihosting::println!("[EXIT]   [STEP 1] Hello from Guest!"); },
                0x51        => unsafe { 
                    semihosting::println!("[EXIT]   [STEP 2a] Mapped addr R/W OK (Stage-2 identity map verified)");
                    let val = core::ptr::read_volatile(0x4780_0000usize as *const u64);
                    semihosting::println!("[EXIT]   [STEP 2a] EL2 cross-check val: {}", val);
                 },
                0x52        => { semihosting::println!("[EXIT]   [STEP 2b] Stage-2 Fault handled, Guest resumed OK (isolation verified)"); },
                0x5F        => { semihosting::println!("[EXIT]   [STEP 2a] FAILED: mapped addr R/W mismatch!"); },
                0x49        => { semihosting::println!("[EXIT]   [STEP 3] IRQ handling done"); },
                _           => {}
            }
            context.elr_el2 += 4;
            return true;
        }
        0x11 => {
           semihosting::println!("[EXIT]   HVC#0x11: Get Hypervisor Info");
            context.regs[0] = 0x48495001; 
            context.elr_el2 += 4;
            return true;
        }
        0x12 => {
            semihosting::println!("[EXIT]   Lagacy Shutdown.");
            return false; 
        }
        0x13 => {
            semihosting::println!("[EXIT]   HVC#0x13: Inject IRQ 32");
            let vbar: u64;
            unsafe { asm!("mrs {}, vbar_el1", out(reg) vbar) };  
            context.vbar_el1 = vbar;
            vgic::inject_irq(vcpu_id, 32);
            context.elr_el2 += 4;
            return true;
        }
        0x14 => {
            semihosting::println!("[EXIT]   HVC#0x14: Inject FIQ 33");
            vgic::inject_fiq(33);
            context.elr_el2 += 4;
            return true;
        }
        0x15 => {
            let intid = context.regs[0] as u32;
            semihosting::println!("[EXIT]   HVC#0x15: IRQ EOI done, INTID= {}", intid as u64);
            context.elr_el2 += 4;
            return true;
        }
        0x20 => {
            unsafe { 
                semihosting::println!("[EXIT] HVC#20 Guest Shutdown...");
                GUEST_SHUTDOWN = true;
            }

            super::hyp::shutdown_guest();
            return false;

        }
        _ => {
           semihosting::println!("[EXIT]   Unknown HVC Number");
            context.elr_el2 += 4;
            return true;
        }
    }
    
    context.elr_el2 += 4; 
    true
}

fn handle_svc(vcpu: &mut Vcpu, info: &VmExitInfo) -> bool {
    let context = vcpu.context_mut();
    let svc_num = context.regs[0];
    semihosting::println!("[EXIT]   SVC Number: {}", svc_num);

    match svc_num {
        0 => {
            semihosting::println!("[EXIT]   SVC#0: Hello from Guest via SVC!");
            context.regs[0] = 0;
        }
        1 => {
            semihosting::println!("[EXIT]   SVC#1: Get Guest ID");
            context.regs[0] = 1;
        }
        _ => {
            semihosting::println!("[EXIT]   Unknown SVC Number");
            context.regs[0] = 0xFFFFFFFF;
        }
    }
    
    context.elr_el2 = info.return_addr as u64;
    
    true
}

pub fn is_guest_shutdown() -> bool {
    unsafe { GUEST_SHUTDOWN }
}

pub fn clear_guest_shutdown() {
    unsafe { GUEST_SHUTDOWN = false; }
}

#[inline]
fn read_esr_el2() -> u64 {
    let esr: u64;
    unsafe {
        asm!("mrs {}, esr_el2", out(reg) esr, options(nostack));
    }
    esr
}

#[inline]
fn read_elr_el2() -> usize {
    let elr: usize;
    unsafe {
        asm!("mrs {}, elr_el2", out(reg) elr, options(nostack));
    }
    elr
}

#[inline]
fn read_spsr_el2() -> u64 {
    let spsr: u64;
    unsafe {
        asm!("mrs {}, spsr_el2", out(reg) spsr, options(nostack));
    }
    spsr
}

#[cfg(test)]
mod tests {
    use super::*;
    use blueos_test_macro::test;

    #[test]
    fn test_parse_exit_reason_exhaustive() {
        assert_eq!(parse_exit_reason(0x16 << 26), VmExitReason::Hvc);
        assert_eq!(parse_exit_reason(0x15 << 26), VmExitReason::Svc);
        assert_eq!(parse_exit_reason(0x24 << 26), VmExitReason::DataAbortLowerEL);
        assert_eq!(parse_exit_reason(0x20 << 26), VmExitReason::InstructionAbortLowerEL);
        assert_eq!(parse_exit_reason(0x01 << 26), VmExitReason::TrappedWfiWfe);
    }

    #[test]
    fn test_handle_hvc_pc_increment_and_psci() {
        let mut vcpu = Vcpu::new(0, 0x4000_0000, 0x4100_0000);
        let initial_pc = 0x4000_1000;
        
        // Scenario 1: HVC #0x11 (Get Information), should resume execution and PC + 4
        vcpu.context_mut().elr_el2 = initial_pc;
        let info_normal = VmExitInfo {
            reason: VmExitReason::Hvc,
            esr: (0x16 << 26) | 0x11,
            far: 0, pstate: 0, return_addr: initial_pc as usize + 4,
        };
        let should_resume = handle_hvc(&mut vcpu, &info_normal);
        assert!(should_resume);
        assert_eq!(vcpu.context().regs[0], 0x48495001, "Should set magic return value");
        assert_eq!(vcpu.context().elr_el2, initial_pc + 4, "PC MUST be incremented to avoid infinite loop!");

        // Scenario 2: PSCI SYSTEM OFF (HVC #0, X0 = 0x84000008), should shut down and refuse recovery.
        vcpu.context_mut().elr_el2 = initial_pc;
        vcpu.context_mut().regs[0] = 0x84000008;
        // EC = 0x16, ISS = 0x00
        let info_shutdown = VmExitInfo {
            reason: VmExitReason::Hvc,
            esr: (0x16 << 26),
            far: 0, pstate: 0, return_addr: initial_pc as usize + 4,
        };
        clear_guest_shutdown();
        let should_resume_shutdown = handle_hvc(&mut vcpu, &info_shutdown);
        assert!(!should_resume_shutdown, "Should refuse to resume on PSCI Shutdown");
        assert!(is_guest_shutdown(), "Global shutdown flag must be set");
        assert_eq!(vcpu.context().elr_el2, initial_pc);
    }
}