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
use super::{vcpu::Vcpu, vgic, hyp, early_uart_print_hex, early_uart_print};

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
    
    // unsafe { early_uart_print("[EXIT] VM Exit Happened!"); }
    // unsafe { early_uart_print_hex("[EXIT]  Reason", reason); }
    // unsafe { early_uart_print_hex("[EXIT]   ESR", esr); }
    // unsafe { early_uart_print_hex("[EXIT]   EC", (esr >> 26) & 0x3F); }
    // unsafe { early_uart_print_hex("[EXIT]   FAR", far); }
    // unsafe { early_uart_print_hex("[EXIT]   PSTATE", pstate); }
    
    // 根据原因处理
    match reason {
        VmExitReason::Hvc => {
            handle_hvc(vcpu, &exit_info)
        }
        VmExitReason::Svc => {
            handle_svc(vcpu, &exit_info)
        }
        VmExitReason::DataAbortLowerEL => {
            unsafe { early_uart_print("[EXIT] Data Abort from Guest (Stage-2 Fault)"); }
            let iss = esr & 0x1FFFFFF;
            let dfsc = iss & 0x3F;
            // unsafe { early_uart_print_hex("[EXIT]   DFSC", dfsc); }
            // unsafe { early_uart_print_hex("[EXIT]   FAR_EL2", {
            //     let far: u64;
            //     core::arch::asm!("mrs {}, far_el2", out(reg) far, options(nostack));
            //     far
            // }); }

            let is_write = (iss & (1 << 6)) != 0; // WnR bit
            
            // ELR_EL2 保存了触发这个 Data Abort 的 Guest 汇编指令地址
            let faulting_pc = vcpu.context().elr_el2; 

            unsafe { 
                early_uart_print("=====================================");
                early_uart_print("[EXIT] PoC Guest triggered Data Abort!");
                early_uart_print_hex("[EXIT]   1. Faulting PC (ELR_EL2) : ", faulting_pc);
                early_uart_print_hex("[EXIT]   2. Target Addr (FAR_EL2) : ", {
                let far: u64;
                core::arch::asm!("mrs {}, far_el2", out(reg) far, options(nostack));
                far
            });
                if is_write {
                    early_uart_print("[EXIT]   3. Access Type           : WRITE");
                } else {
                    early_uart_print("[EXIT]   3. Access Type           : READ");
                }
                early_uart_print_hex("[EXIT]   4. DFSC Code             : ", dfsc as u64);
                early_uart_print("=====================================");
            }

            if (dfsc & 0x3C) == 0x04 || (dfsc & 0x3C) == 0x08 || (dfsc & 0x3C) == 0x0C {
                // Translation fault (level 0/1/2/3) - Stage-2 未映射
                unsafe { early_uart_print("[EXIT]   Stage-2 Translation Fault - skipping instruction"); }
                // 跳过触发 Fault 的指令，让 Guest 继续
                vcpu.context_mut().elr_el2 += 4;
                return true;
            }
            unsafe { early_uart_print("[EXIT]   Unrecoverable Data Abort, terminating Guest"); }
            false
        }
        VmExitReason::InstructionAbortLowerEL => {
            unsafe { early_uart_print("[EXIT] Instruction Abort from Guest!"); }
            let iss = esr & 0x1FFFFFF;
            let ifsc = iss & 0x3F;
            
            if (ifsc & 0x3C) == 0x14 {
                 unsafe { early_uart_print("[EXIT]   Stage-2 Translation Fault (Instruction)!"); }
            }
            //  unsafe { early_uart_print_hex("[EXIT]   Fault Address (FAR): ", far); }
            //  unsafe { early_uart_print("[EXIT]   Terminate Guest"); }
             false
        }
        VmExitReason::TrappedWfiWfe => {
            unsafe { early_uart_print("[EXIT] Trapped WFI/WFE instruction"); }
            vcpu.context_mut().elr_el2 += 4;
            true
        }
        VmExitReason::Unknown(ec) => {
            unsafe { early_uart_print("[EXIT]  Unknown Exit Reason: EC = "); }
            false
        }
    }
}

fn handle_hvc(vcpu: &mut Vcpu, info: &VmExitInfo) -> bool {
    let saved_x0 = vcpu.context().regs[0];
    unsafe { early_uart_print("[EXIT] Handle HVC Call"); }
    let context = vcpu.context_mut();
    let hvc_num = info.esr & 0xFFFF;
    unsafe { early_uart_print_hex("[EXIT]   HVC#", hvc_num); }
    
    // Easy HVC Services.
    match hvc_num {
        0x01 => {
            unsafe { early_uart_print_hex("[EXIT]   ESR_EL1", context.regs[0]); }
            // elr_el2 point to Guest Sync Handler hvc #1 
            // after +4, "eret" back to "eret" of Sync Handler，so that "eret" will trigger next instruction of Fault.
            context.elr_el2 += 4;
            return true;
        }
        0x10 => {
            unsafe { early_uart_print_hex("[EXIT] HVC#0x10: Guest status, x0=", context.regs[0]); }
            match context.regs[0] {
                0xDEAD_BEEF => unsafe { early_uart_print("[EXIT]   [STEP 1] Hello from Guest!"); },
                0x51        => unsafe { 
                    early_uart_print("[EXIT]   [STEP 2a] Mapped addr R/W OK (Stage-2 identity map verified)");
                    // 可选：EL2 侧交叉验证（identity map 下 IPA == PA）
                    let val = core::ptr::read_volatile(0x4780_0000usize as *const u64);
                    early_uart_print_hex("[EXIT]   [STEP 2a] EL2 cross-check val", val);
                 },
                0x52        => unsafe { early_uart_print("[EXIT]   [STEP 2b] Stage-2 Fault handled, Guest resumed OK (isolation verified)"); },
                0x5F        => unsafe { early_uart_print("[EXIT]   [STEP 2a] FAILED: mapped addr R/W mismatch!"); },
                0x49        => unsafe { early_uart_print("[EXIT]   [STEP 3] IRQ handling done"); },
                _           => {}
            }
            context.elr_el2 += 4;
            return true;
        }
        0x11 => {
            unsafe { early_uart_print("[EXIT]   HVC#0x11: Get Hypervisor Info"); }
            context.regs[0] = 0x48495001; 
            context.elr_el2 += 4;
            return true;
        }
        0x12 => {
            unsafe { early_uart_print("[EXIT]   Lagacy Shutdown."); }
            return false; 
        }
        0x13 => {
            unsafe { early_uart_print("[EXIT]   HVC#0x13: Inject IRQ 32"); }
            let vbar: u64;
            unsafe { asm!("mrs {}, vbar_el1", out(reg) vbar) };
            // unsafe { early_uart_print_hex("[EXIT]   HVC#0x13: Current VBAR_EL1=", current_vbar); }
            // unsafe { early_uart_print_hex("[EXIT]   HVC#0x13: SPSR_EL2=", info.pstate); }   
            
            // TO Fix:Nowaday, we can't set vabar in EL1. So force set VBAR_EL1 to see if it works.
            // let target_vbar: u64 = 0x48000800;
            // unsafe { asm!("msr vbar_el1, {}", in(reg) target_vbar) };
            // IMPORTANT: Update the context as well, otherwise restore_context 
            // will overwrite the hardware register with the old (0) value!
            context.vbar_el1 = vbar;
            vgic::inject_irq(32);
            context.elr_el2 += 4;
            return true;
        }
        0x14 => {
            unsafe { early_uart_print("[EXIT]   HVC#0x14: Inject FIQ 33"); }
            vgic::inject_fiq(33);
            context.elr_el2 += 4;
            return true;
        }
        0x15 => {
            let intid = context.regs[0] as u32;
            unsafe { early_uart_print_hex("[EXIT]   HVC#0x15: IRQ EOI done, INTID=", intid as u64); }
            context.elr_el2 += 4;
            return true;
        }
        0x20 => {
            unsafe { 
                early_uart_print("[EXIT] HVC#20 Guest Shutdown...");
                GUEST_SHUTDOWN = true;
            }

            super::hyp::shutdown_guest();
            return false;

        }
        _ => {
            unsafe { early_uart_print("[EXIT]   Unknown HVC Number"); }
            context.elr_el2 += 4;
            return true;
        }
    }

    // Do NOT restore x0, as it might be used for return value.
    // if hvc_num != 1 {
    //    context.regs[0] = saved_x0;
    // }
    
    context.elr_el2 += 4;
    
    true
}

fn handle_svc(vcpu: &mut Vcpu, info: &VmExitInfo) -> bool {
    let context = vcpu.context_mut();
    let svc_num = context.regs[0];
    // unsafe { early_uart_print_hex("[EXIT]   SVC Number", svc_num); }
    
    match svc_num {
        0 => {
            unsafe { early_uart_print("[EXIT]   SVC#0: Hello from Guest via SVC!"); }
            context.regs[0] = 0;
        }
        1 => {
            unsafe { early_uart_print("[EXIT]   SVC#1: Get Guest ID"); }
            context.regs[0] = 1;
        }
        _ => {
            unsafe { early_uart_print("[EXIT]   Unknown SVC Number"); }
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
    
    #[test]
    fn test_parse_exit_reason() {
        let hvc_esr = 0x56000000;
        assert_eq!(parse_exit_reason(hvc_esr), VmExitReason::Hvc);
        let svc_esr = 0x55000000;
        assert_eq!(parse_exit_reason(svc_esr), VmExitReason::Svc);
    }
    
    #[test]
    fn test_vcpu_context_access() {
        let mut vcpu = Vcpu::new(0, 0x4000_0000, 0x4100_0000);

        {
            let context = vcpu.context_mut();
            context.regs[0] = 0x12345678;
        }
        
        let context = vcpu.context();
        assert_eq!(context.regs[0], 0x12345678);
    }
}