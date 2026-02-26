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
pub mod vgic;
pub mod vector;

pub use vcpu::{Vcpu, VcpuManager, VcpuState};
pub use exit::{VmExitReason, VmExitInfo};
pub use hyp::{hyp_init, get_current_el};
use log::info;

#[no_mangle]
pub extern "C" fn trap_irq(_context: &mut crate::arch::aarch64::Context) -> usize {
    info!("[EL2] IRQ trap");
    0
}

#[no_mangle]
pub extern "C" fn trap_fiq(_context: &mut crate::arch::aarch64::Context) -> usize {
    info!("[EL2] FIQ trap");
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
    info!("[VIRT] Starting virt_init...");
    let el = get_current_el();
    info!("[VIRT] Current EL: {}", el);
    hyp_init();
    info!("[VIRT] virt_init complete!");
}

pub fn virt_start_guest() {  
    let entry = guest::guest_entry as usize;
    let stack_top = guest::GUEST_STACK_TOP;
    
    unsafe {
        let vcpu = VCPU_MANAGER.0.create_vcpu(0, entry, stack_top)
            .expect("Failed to create vCPU");
        VCPU_MANAGER.0.run_vcpu(0);
    }
}