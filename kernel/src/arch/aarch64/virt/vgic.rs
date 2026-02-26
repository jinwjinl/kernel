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
use super::VCPU_MANAGER;

const MAX_LR: usize = 4;
const MAX_PENDING: usize = 64;
const MAX_VCPUS: usize = 4;

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct Vgic {
    pending_irqs: [u32; MAX_PENDING],
    pending_count: usize,
    pending_head: usize,
    pending_tail: usize,
}

impl Vgic {
    pub const fn new() -> Self {
        Self {
            pending_irqs: [0; MAX_PENDING],
            pending_count: 0,
            pending_head: 0,
            pending_tail: 0,
        }
    }
}

// Global VGIC state for each vCPU
static mut VGIC_CPU_STATES: [Vgic; MAX_VCPUS] = [Vgic::new(); MAX_VCPUS];

// Per-CPU Initialization (called by vCPU on first run)
pub fn cpu_init(vcpu_id: usize) {
    unsafe {
        // Force reset VGIC state to ensure clean memory
        if vcpu_id < MAX_VCPUS {
            let vgic = &mut VGIC_CPU_STATES[vcpu_id];
            vgic.pending_head = 0;
            vgic.pending_tail = 0;
            vgic.pending_count = 0;
            // No need to zero pending_irqs as we use head/tail
        }

        // 1. Enable System Register access for EL2 (ICC_SRE_EL2)
        let mut sre: u64;
        asm!("mrs {}, ICC_SRE_EL2", out(reg) sre);
        if (sre & 0x9) != 0x9 { 
             sre |= 0x9; 
             asm!("msr ICC_SRE_EL2, {}", in(reg) sre);
             asm!("isb");
        }

        // 2. Enable vGIC
        let hcr: u64 = 1; 
        asm!("msr S3_4_C12_C11_0, {}", in(reg) hcr);

        // 3. Configure VMCR (Group 0/1 Enable)
        let vmcr: u64 = 0x3;
        asm!("msr S3_4_C12_C11_7, {}", in(reg) vmcr);
        
        // Clear all LRs
        for i in 0..MAX_LR {
             write_lr(i, 0);
        }
    }
}

pub fn inject(vcpu_id: usize, intid: u32) {
    unsafe {
        if vcpu_id >= MAX_VCPUS { return; }
        let vgic = &mut VGIC_CPU_STATES[vcpu_id];
        
        if vgic.pending_count >= MAX_PENDING {
            info!("[VGIC] Error: Queue full, drop IRQ {}", intid);
            return;
        }
        
        vgic.pending_irqs[vgic.pending_tail] = intid;
        vgic.pending_tail = (vgic.pending_tail + 1) % MAX_PENDING;
        vgic.pending_count += 1;
    }
}

pub fn flush(vcpu_id: usize) {
        unsafe {
            if vcpu_id >= MAX_VCPUS { return; }
            let vgic = &mut VGIC_CPU_STATES[vcpu_id];
            
            // Sanity check for memory corruption
            if vgic.pending_head >= MAX_PENDING || vgic.pending_tail >= MAX_PENDING {
                info!("[VGIC] Corruption detected! Resetting state. head={:#x} tail={:#x}", vgic.pending_head, vgic.pending_tail);
                vgic.pending_head = 0;
                vgic.pending_tail = 0;
                vgic.pending_count = 0;
            }

            // Debug print to diagnose corruption
            if vgic.pending_count > 0 {
                 info!("[VGIC] Flush: head={} tail={} count={}", 
                          vgic.pending_head, vgic.pending_tail, vgic.pending_count);
            }

            let mut elrsr: u64;
        asm!("mrs {}, S3_4_C12_C11_5", out(reg) elrsr);
        
        // Force clean Active LRs to prevent deadlock
        // If an LR is Active (State=2), it means Guest has Acked it.
        // If it sticks there, it blocks lower/same priority interrupts.
        for i in 0..MAX_LR {
            if (elrsr & (1 << i)) == 0 {
                let lr = read_lr(i);
                let state = (lr >> 62) & 0x3;
                if state == 2 { // Active
                    info!("[VGIC] Force clearing Active LR{} (IRQ {})", i, lr & 0x3FF);
                    write_lr(i, 0);
                }
            }
        }
        
        // Re-read ELRSR
        asm!("mrs {}, S3_4_C12_C11_5", out(reg) elrsr);
        
        // Simple Flush: Fill available LRs
        let mut free_mask = elrsr & 0xF;
        for i in 0..MAX_LR {
            if (free_mask & (1 << i)) != 0 && vgic.pending_count > 0 {
                let intid = vgic.pending_irqs[vgic.pending_head];
                vgic.pending_head = (vgic.pending_head + 1) % MAX_PENDING;
                vgic.pending_count -= 1;
                
                let lr_val: u64 = (1 << 62) | (1 << 60) | (intid as u64);
                
                // For Timer IRQ (27), enable HW bit for physical forwarding
                // This links the virtual interrupt EOI to the physical interrupt EOI
                // if intid == 27 {
                //    lr_val |= 1 << 61; // HW = 1
                //    lr_val |= (intid as u64) << 32; // pINTID = 27
                // }
                
                write_lr(i, lr_val);
                info!("[VGIC] Flushed IRQ {} to LR{}", intid, i);
                free_mask &= !(1 << i);
            }
        }
    }
}

pub fn sync(_vcpu_id: usize) {
    // Post-exit sync (update pending state based on LRs if needed)
}

// Global helper for external calls
pub fn init() {
    // Global Distributor init (if any)
}

pub fn inject_irq(intid: u32) {
    unsafe {
        if let Some(id) = VCPU_MANAGER.0.current_vcpu_id() {
            inject(id, intid);
        }
    }
}

pub fn inject_fiq(_intid: u32) {
    // ...
}

unsafe fn read_lr(index: usize) -> u64 {
    let val: u64;
    match index {
        0 => asm!("mrs {}, S3_4_C12_C12_0", out(reg) val),
        1 => asm!("mrs {}, S3_4_C12_C12_1", out(reg) val),
        2 => asm!("mrs {}, S3_4_C12_C12_2", out(reg) val),
        3 => asm!("mrs {}, S3_4_C12_C12_3", out(reg) val),
        _ => val = 0,
    }
    val
}

unsafe fn write_lr(index: usize, val: u64) {
    match index {
        0 => asm!("msr S3_4_C12_C12_0, {}", in(reg) val),
        1 => asm!("msr S3_4_C12_C12_1, {}", in(reg) val),
        2 => asm!("msr S3_4_C12_C12_2, {}", in(reg) val),
        3 => asm!("msr S3_4_C12_C12_3, {}", in(reg) val),
        _ => (),
    }
}