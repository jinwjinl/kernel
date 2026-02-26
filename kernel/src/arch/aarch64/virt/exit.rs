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

use log::info;
use core::arch::asm;
use super::vcpu::Vcpu;
use super::vgic;

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
    let far = read_elr_el2();
    let pstate = read_spsr_el2();
    let reason = parse_exit_reason(esr);

    let exit_info = VmExitInfo {
        reason,
        esr,
        far,
        pstate,
        return_addr: far + 4,  
    };
    
    info!("[EXIT] VM Exit Happened!");
    info!("[EXIT]  Reason: {:?}", reason);
    info!("[EXIT]   ESR: {:#x}", esr);
    info!("[EXIT]   EC: {:#x}", (esr >> 26) & 0x3F);
    info!("[EXIT]   FAR: {:#x}", far);
    info!("[EXIT]   PSTATE: {:#x}", pstate);
    
    // 根据原因处理
    match reason {
        VmExitReason::Hvc => {
            handle_hvc(vcpu, &exit_info)
        }
        VmExitReason::Svc => {
            handle_svc(vcpu, &exit_info)
        }
        VmExitReason::DataAbortLowerEL => {
            info!("[EXIT] Data Abort from Guest!");
            // 解析 ISS
            let iss = esr & 0x1FFFFFF;
            let dfsc = iss & 0x3F;
            info!("[EXIT]   ISS: {:#x}, DFSC: {:#x}", iss, dfsc);
            
            // 检查是否是 Stage-2 Fault
            if (dfsc & 0x3C) == 0x14 {
                info!("[EXIT]   Stage-2 Translation Fault!");
            }
            
            info!("[EXIT]   Fault Address (FAR): {:#x}", far);
            info!("[EXIT]   Terminate Guest");
            false
        }
        VmExitReason::InstructionAbortLowerEL => {
            info!("[EXIT] Instruction Abort from Guest!");
            let iss = esr & 0x1FFFFFF;
            let ifsc = iss & 0x3F;
            info!("[EXIT]   ISS: {:#x}, IFSC: {:#x}", iss, ifsc);
            
            if (ifsc & 0x3C) == 0x14 {
                 info!("[EXIT]   Stage-2 Translation Fault (Instruction)!");
            }
             info!("[EXIT]   Fault Address (FAR): {:#x}", far);
             info!("[EXIT]   Terminate Guest");
             false
        }
        VmExitReason::TrappedWfiWfe => {
            info!("[EXIT] Trapped WFI/WFE instruction");
            vcpu.context_mut().elr_el2 += 4;
            true
        }
        VmExitReason::Unknown(ec) => {
            info!("[EXIT]  Unknown Exit Reason: EC = {:#x}", ec);
            info!("[EXIT]  Terminate Guest");
            false
        }
    }
}

fn handle_hvc(vcpu: &mut Vcpu, info: &VmExitInfo) -> bool {
    let saved_x0 = vcpu.context().regs[0];
    info!("[EXIT] Handle HVC Call");
    let context = vcpu.context_mut();
    let hvc_num = info.esr & 0xFFFF;
    info!("[EXIT]   HVC Number: {}", hvc_num);
    
    // Easy HVC Services.
    match hvc_num {
        0 => {
            info!("[EXIT]   HVC#0: Hello from Hypervisor! x0={:#x}", context.regs[0]);
            
            if context.regs[0] == 0xeeee {
                 info!("[EXIT]   EOI Done. SPSR_EL2={:#x} ELR_EL2={:#x}", 
                          context.spsr, context.elr_el2);
            }
            if context.regs[0] == 0xAAAA {
                info!("[EXIT]   IRQ Handler Entry. ELR_EL1={:#x}", context.regs[1]);
            }
        }
        1 => {
            info!("[EXIT]   HVC#1: Get Hypervisor Info");
            context.regs[0] = 0x48495001; 
        }
        2 => {
            info!("[EXIT]   HVC#2: Shutdown Request");
            info!("[EXIT]   Terminate Guest (Shutdown)");
            return false; 
        }
        3 => {
            info!("[EXIT]   HVC#3: Inject IRQ 32");
            let current_vbar: u64;
            unsafe { asm!("mrs {}, vbar_el1", out(reg) current_vbar) };
            info!("[EXIT]   HVC#3: Current VBAR_EL1={:#x}", current_vbar);
            info!("[EXIT]   HVC#3: SPSR_EL2={:#x}", info.pstate);   
            
            // TO Fix:Nowaday, we can't set vabar in EL1. So force set VBAR_EL1 to see if it works.
            let target_vbar: u64 = 0x48000800;
            unsafe { asm!("msr vbar_el1, {}", in(reg) target_vbar) };
            // IMPORTANT: Update the context as well, otherwise restore_context 
            // will overwrite the hardware register with the old (0) value!
            context.vbar_el1 = target_vbar;
            info!("[EXIT]   HVC#3: Forced VBAR_EL1 to {:#x}", target_vbar);
            vgic::inject_irq(32);
        }
        4 => {
            info!("[EXIT]   HVC#4: Inject FIQ 33");
            vgic::inject_fiq(33);
        }
        5 => {
            info!("[EXIT]   HVC#5: Guest is about to access unmapped memory (0x8000_0000)");
            info!("[EXIT]   Expect Data Abort from Guest...");
        }
        _ => {
            info!("[EXIT]   Unknown HVC Number: {}", hvc_num);
        }
    }

    if hvc_num != 1 {
        context.regs[0] = saved_x0;
    }
    
    let old_elr = context.elr_el2;
    context.elr_el2 += 4;
    info!("[EXIT]   Advancing ELR: {:#x} -> {:#x}", old_elr, context.elr_el2);
    
    true
}

fn handle_svc(vcpu: &mut Vcpu, info: &VmExitInfo) -> bool {
    info!("[EXIT] Handling SVC Call");
    let context = vcpu.context_mut();
    let svc_num = context.regs[0];
    info!("[EXIT]   SVC Number: {}", svc_num);
    
    match svc_num {
        0 => {
            info!("[EXIT]   SVC#0: Hello from Guest via SVC!");
            context.regs[0] = 0;
        }
        1 => {
            info!("[EXIT]   SVC#1: Get Guest ID");
            context.regs[0] = 1;
        }
        _ => {
            info!("[EXIT]   Unknown SVC Number: {}", svc_num);
            context.regs[0] = 0xFFFFFFFF;
        }
    }
    
    context.elr_el2 = info.return_addr as u64;
    
    true
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