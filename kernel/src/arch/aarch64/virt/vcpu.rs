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
use super::hyp::read_hcr_el2;

/// HCR_EL2_VI: Enable virtual IRQ.
const HCR_EL2_VI: u64 = 1 << 7;
/// HCR_EL2_VF: Enable virtual FIQ.
const HCR_EL2_VF: u64 = 1 << 6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VcpuState {
    Stopped,
    Running,
    Paused,
    Exited,
}

/// Using #[repr(C)] to ensure compatibility with ARM AAPCS64 calling convention,
/// using #[repr(align(16))] to ensure 16-byte alignment, satisfying SIMD register alignment requirements
#[derive(Debug, Default)]
#[repr(C)]
#[repr(align(16))]
pub struct VcpuStateStruct {
    pub regs: [u64; 31],
    pub elr_el2: u64,
    pub sp: u64,
    pub pstate: u64,
    pub spsr: u64,
    pub vbar_el1: u64,
}

impl VcpuStateStruct {
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
    
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.elr_el2 != 0 && self.sp != 0
    }
    
    #[inline]
    pub fn reset(&mut self) {
        self.regs = [0; 31];
        self.elr_el2 = 0;
        self.sp = 0;
        self.pstate = 0;
        self.spsr = 0;
        self.vbar_el1 = 0;
    }
    
    #[inline]
    pub fn elr(&self) -> u64 {
        self.elr_el2
    }
    
    #[inline]
    pub fn set_elr(&mut self, elr: u64) {
        self.elr_el2 = elr;
    }
    
    #[inline]
    pub fn spsr(&self) -> u64 {
        self.spsr
    }
    
    #[inline]
    pub fn set_spsr(&mut self, spsr: u64) {
        self.spsr = spsr;
    }
}

#[repr(align(16))]
pub struct Vcpu {
    id: usize,
    state: VcpuState,
    context: VcpuStateStruct,
    entry: usize,
    stack_top: usize,
    pending_irq: bool,
    pending_fiq: bool,
}

impl Vcpu {
    #[inline]
    pub fn new(id: usize, entry: usize, stack_top: usize) -> Self {
        let mut context = VcpuStateStruct::new();
        context.elr_el2 = entry as u64;
        context.spsr = 0x3C5; // Default PSTATE: EL1h, DAIF masked
        
        Self {
            id,
            state: VcpuState::Stopped,
            context,
            entry,
            stack_top,
            pending_irq: false,
            pending_fiq: false,
        }
    }
    
    #[inline]
    pub fn id(&self) -> usize {
        self.id
    }
    
    #[inline]
    pub fn entry_point(&self) -> usize {
        self.entry
    }
    
    #[inline]
    pub fn stack_top(&self) -> usize {
        self.stack_top
    }
    
    #[inline]
    pub fn state(&self) -> VcpuState {
        self.state
    }
    
    #[inline]
    pub fn context(&self) -> &VcpuStateStruct {
        &self.context
    }
    
    #[inline]
    pub fn context_mut(&mut self) -> &mut VcpuStateStruct {
        &mut self.context
    }

    #[inline]
    pub fn pending_irq(&self) -> bool {
        self.pending_irq
    }

    #[inline]
    pub fn pending_fiq(&self) -> bool {
        self.pending_fiq
    }

    #[inline]
    pub fn set_pending_irq(&mut self, val: bool) {
        self.pending_irq = val;
    }

    #[inline]
    pub fn set_pending_fiq(&mut self, val: bool) {
        self.pending_fiq = val;
    }
    
    #[inline]
    pub fn elr(&self) -> u64 {
        self.context.elr_el2
    }
    
    #[inline]
    pub fn set_entry(&mut self, entry: usize) {
        self.entry = entry;
    }
    
    #[inline]
    pub fn set_stack_top(&mut self, stack_top: usize) {
        self.stack_top = stack_top;
    }
    
    #[inline]
    pub fn set_state(&mut self, state: VcpuState) {
        self.state = state;
    }
    
    pub fn save_context(&mut self) {
        let x0_value: u64;
        unsafe {
            asm!("mov {}, x0", out(reg) x0_value, options(nostack));
        }

        self.context.regs[0] = x0_value;
        
        unsafe {
            asm!(
                "stp x1,  x2,  [x0, #8]",
                "stp x3,  x4,  [x0, #24]",
                "stp x5,  x6,  [x0, #40]",
                "stp x7,  x8,  [x0, #56]",
                "stp x9,  x10, [x0, #72]",
                "stp x11, x12, [x0, #88]",
                "stp x13, x14, [x0, #104]",
                "stp x15, x16, [x0, #120]",
                "stp x17, x18, [x0, #136]",
                "stp x19, x20, [x0, #152]",
                "stp x21, x22, [x0, #168]",
                "stp x23, x24, [x0, #184]",
                "stp x25, x26, [x0, #200]",
                "stp x27, x28, [x0, #216]",
                "stp x29, x30, [x0, #232]",
                in("x0") &mut self.context.regs as *mut u64,
                options(nostack)
            );
        }
        
        self.context.elr_el2 = Self::read_elr_el2();
        self.context.sp      = Self::read_sp();
        self.context.pstate  = Self::read_pstate();
        self.context.spsr    = Self::read_spsr_el2();
        self.context.vbar_el1 = Self::read_vbar_el1();
    }
    
    pub fn restore_context(&mut self) {
        // Note: ELR_EL1 and SPSR_EL1 are Guest registers. 
        // We should NOT overwrite them with ELR_EL2/SPSR_EL2.
        
        // Self::write_elr_el1(self.context.elr_el2); // REMOVED: Incorrect
        Self::write_sp(self.context.sp);
        // Self::write_pstate(self.context.pstate); // REMOVED: Incorrect (Affects EL2)
        // Self::write_spsr_el1(self.context.spsr); // REMOVED: Incorrect
        Self::write_vbar_el1(self.context.vbar_el1);
        
        unsafe {
            asm!(
                "ldp x1,  x2,  [x0, #8]",
                "ldp x3,  x4,  [x0, #24]",
                "ldp x5,  x6,  [x0, #40]",
                "ldp x7,  x8,  [x0, #56]",
                "ldp x9,  x10, [x0, #72]",
                "ldp x11, x12, [x0, #88]",
                "ldp x13, x14, [x0, #104]",
                "ldp x15, x16, [x0, #120]",
                "ldp x17, x18, [x0, #136]",
                "ldp x19, x20, [x0, #152]",
                "ldp x21, x22, [x0, #168]",
                "ldp x23, x24, [x0, #184]",
                "ldp x25, x26, [x0, #200]",
                "ldp x27, x28, [x0, #216]",
                "ldp x29, x30, [x0, #232]",
                in("x0") &self.context.regs as *const u64,
                options(nostack)
            );
        }
        
        unsafe {
            asm!("mov x0, {}", in(reg) self.context.regs[0], options(nostack));
        }
    }

    pub fn restore_regs(&self) {
        unsafe {
            asm!(
                "ldp x1,  x2,  [x0, #8]",
                "ldp x3,  x4,  [x0, #24]",
                "ldp x5,  x6,  [x0, #40]",
                "ldp x7,  x8,  [x0, #56]",
                "ldp x9,  x10, [x0, #72]",
                "ldp x11, x12, [x0, #88]",
                "ldp x13, x14, [x0, #104]",
                "ldp x15, x16, [x0, #120]",
                "ldp x17, x18, [x0, #136]",
                "ldp x19, x20, [x0, #152]",
                "ldp x21, x22, [x0, #168]",
                "ldp x23, x24, [x0, #184]",
                "ldp x25, x26, [x0, #200]",
                "ldp x27, x28, [x0, #216]",
                "ldp x29, x30, [x0, #232]",
                in("x0") &self.context.regs as *const u64,
                options(nostack)
            );
            asm!("mov x0, {}", in(reg) self.context.regs[0], options(nostack));
        }
    }
    
    pub fn run(&mut self) {
        if self.state == VcpuState::Stopped {
            // vgic::cpu_init(self.id);
        }
        self.state = VcpuState::Running;
        
        // Flush pending IRQs to LRs（留空）
        // vgic::flush(self.id);
        
        let hcr = read_hcr_el2();
        
        if self.pending_irq {
            // vgic::inject_irq(32);
        }
        if self.pending_fiq {
            // vgic::inject_fiq(33);
        }

        let sctlr_el1 = 0x30D00800u64;
        let ctx_ptr = &self.context as *const VcpuStateStruct;
        
        unsafe {
            asm!(
                // 1. Configure System Registers
                "msr sctlr_el1, {sctlr}",
                "msr sp_el1, {sp}",
                "msr elr_el2, {elr}",
                "msr spsr_el2, {spsr}",
                
                // 2. Restore General Purpose Registers (x1-x30)
                "ldp x1,  x2,  [x0, #8]",
                "ldp x3,  x4,  [x0, #24]",
                "ldp x5,  x6,  [x0, #40]",
                "ldp x7,  x8,  [x0, #56]",
                "ldp x9,  x10, [x0, #72]",
                "ldp x11, x12, [x0, #88]",
                "ldp x13, x14, [x0, #104]",
                "ldp x15, x16, [x0, #120]",
                "ldp x17, x18, [x0, #136]",
                "ldp x19, x20, [x0, #152]",
                "ldp x21, x22, [x0, #168]",
                "ldp x23, x24, [x0, #184]",
                "ldp x25, x26, [x0, #200]",
                "ldp x27, x28, [x0, #216]",
                "ldp x29, x30, [x0, #232]",
                
                // 3. Restore x0 last
                "ldr x0, [x0, #0]",
                
                "dsb ish",
                "isb sy",
                "eret",
                
                sctlr = in(reg) sctlr_el1,
                sp = in(reg) self.stack_top,
                elr = in(reg) self.context.elr_el2,
                spsr = in(reg) self.context.spsr,
                in("x0") ctx_ptr,
                options(noreturn)
            );
        }
    }
    
    pub fn inject_irq(&mut self) {
        self.pending_irq = true;
        // TODO: achieve inject_irq
    }

    pub fn inject_fiq(&mut self) {
        self.pending_fiq = true;
        // TODO: achieve inject_fiq
    }

    #[inline]
    pub fn can_run(&self) -> bool {
        self.state == VcpuState::Stopped || 
        self.state == VcpuState::Paused ||
        self.state == VcpuState::Exited
    }
    

    #[inline]
    fn read_elr_el2() -> u64 {
        let elr: u64;
        unsafe {
            asm!("mrs {}, elr_el2", out(reg) elr, options(nostack));
        }
        elr
    }
    

    #[inline]
    fn write_elr_el1(elr: u64) {
        unsafe {
            asm!("msr elr_el1, {}", in(reg) elr, options(nostack, nomem));
        }
    }
    
    #[inline]
    fn read_sp() -> u64 {
        let sp: u64;
        unsafe {
            asm!("mov {}, sp", out(reg) sp, options(nostack));
        }
        sp
    }
    
    #[inline]
    fn write_sp(sp: u64) {
        unsafe {
            asm!("mov sp, {}", in(reg) sp, options(nostack, nomem));
        }
    }

    #[inline]
    fn read_pstate() -> u64 {
        let pstate: u64;
        unsafe {
            asm!("mrs {}, daif", out(reg) pstate, options(nostack));
        }
        pstate
    }
    
    #[inline]
    fn write_pstate(pstate: u64) {
        unsafe {
            asm!("msr daif, {}", in(reg) pstate, options(nostack, nomem));
        }
    }
    
    #[inline]
    fn read_spsr_el2() -> u64 {
        let spsr: u64;
        unsafe {
            asm!("mrs {}, spsr_el2", out(reg) spsr, options(nostack));
        }
        spsr
    }
    
    #[inline]
    fn write_spsr_el1(spsr: u64) {
        unsafe {
            asm!("msr spsr_el1, {}", in(reg) spsr, options(nostack, nomem));
        }
    }

    #[inline]
    fn read_vbar_el1() -> u64 {
        let vbar: u64;
        unsafe {
            asm!("mrs {}, vbar_el1", out(reg) vbar, options(nostack));
        }
        vbar
    }

    #[inline]
    fn write_vbar_el1(vbar: u64) {
        unsafe {
            asm!("msr vbar_el1, {}", in(reg) vbar, options(nostack, nomem));
        }
    }
}

#[repr(align(16))]
pub struct VcpuManager {
    vcpus: [Option<Vcpu>; 4],
    count: usize,
    current_vcpu: Option<usize>,
}

impl VcpuManager {
    #[inline]
    pub const fn new() -> Self {
        Self {
            vcpus: [None, None, None, None],
            count: 0,
            current_vcpu: None,
        }
    }
    
    pub fn create_vcpu(
        &mut self, 
        id: usize, 
        entry: usize, 
        stack_top: usize
    ) -> Result<&mut Vcpu, &'static str> {
        if id >= 4 {
            return Err("vCPU ID out of range (max 3)");
        }

        // Temporary check for max vCPU count
        if self.count >= 4 {
            return Err("Reached max vCPU count");
        }
        
        if self.vcpus[id].is_some() {
            return Err("vCPU ID already used");
        }
        
        let vcpu = Vcpu::new(id, entry, stack_top);
        self.vcpus[id] = Some(vcpu);
        self.count += 1;
        
        Ok(self.vcpus[id].as_mut().unwrap())
    }
    
    pub fn run_vcpu(&mut self, vcpu_id: usize) {
        let vcpu = match self.vcpus[vcpu_id].as_mut() {
            Some(v) => v,
            None => {
                info!("[VCPU] Error: vCPU {} not found", vcpu_id);
                return;
            }
        };
        
        if !vcpu.can_run() {
            info!("[VCPU] Warning: vCPU {} state is {:?}, cannot run", 
                     vcpu_id, vcpu.state());
            return;
        }
        
        info!("[VCPU] Running vCPU {}", vcpu.id());
        info!("[VCPU]   Entry: {:#018x}", vcpu.entry_point());
        info!("[VCPU]   Stack Top: {:#018x}", vcpu.stack_top());
        
        self.current_vcpu = Some(vcpu_id);
        vcpu.run();
    }
    
    #[inline]
    pub fn get_vcpu(&mut self, id: usize) -> Option<&mut Vcpu> {
        self.vcpus[id].as_mut()
    }
    
    #[inline]
    pub fn current_vcpu_id(&self) -> Option<usize> {
        self.current_vcpu
    }
    
    #[inline]
    pub fn vcpu_count(&self) -> usize {
        self.count
    }
    
    #[inline]
    pub fn has_running(&self) -> bool {
        self.current_vcpu.is_some()
    }
    
    pub fn iter(&mut self) -> impl Iterator<Item = (usize, &mut Vcpu)> {
        self.vcpus
            .iter_mut()
            .enumerate()
            .filter_map(|(id, vcpu)| vcpu.as_mut().map(|v| (id, v)))
    }
}

#[derive(Debug)]
pub enum VcpuError {
    IdOutOfRange,
    MaxLimitReached,
    IdAlreadyUsed,
    NotFound,
    InvalidState,
}

impl core::fmt::Display for VcpuError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            VcpuError::IdOutOfRange => 
                write!(f, "vCPU ID out of range (max 3)"),
            VcpuError::MaxLimitReached => 
                write!(f, "Reached max vCPU count"),
            VcpuError::IdAlreadyUsed => 
                write!(f, "vCPU ID already used"),
            VcpuError::NotFound => 
                write!(f, "vCPU not found"),
            VcpuError::InvalidState => 
                write!(f, "vCPU state invalid for this operation"),
        }
    }
}