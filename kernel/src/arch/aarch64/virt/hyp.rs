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

use tock_registers::interfaces::{Readable, Writeable};
use log::info;
use crate::arch::aarch64::{
    registers::hcr_el2::HCR_EL2,
    registers::sctlr_el2::SCTLR_EL2,
    registers::spsr_el2::SPSR_EL2,
    virt::vector
};

// const CNTHCTL_EL2_ADDR: usize = 0x80_0000_0014;
// const CNTVOFF_EL2_ADDR: usize = 0x80_0000_0018;

#[inline]
pub fn get_current_el() -> u64 {
    let current_el: u64;
    unsafe {
        core::arch::asm!("mrs {}, currentel", out(reg) current_el);
    }
    (current_el >> 2) & 0x3
}

#[inline]
pub fn read_hcr_el2() -> u64 {
    HCR_EL2.get()
}

#[inline]
pub fn write_hcr_el2(val: u64) {
    HCR_EL2.set(val);
}

#[inline]
pub fn read_vbar_el2() -> u64 {
    let vbar: u64;
    unsafe {
        core::arch::asm!("mrs {}, vbar_el2", out(reg) vbar);
    }
    vbar
}

#[inline]
pub fn write_vbar_el2(val: u64) {
    unsafe {
        core::arch::asm!("msr vbar_el2, {}", in(reg) val);
    }
}

#[inline]
pub fn read_esr_el2() -> u64 {
    let esr: u64;
    unsafe {
        core::arch::asm!("mrs {}, esr_el2", out(reg) esr);
    }
    esr
}

#[inline]
pub fn read_elr_el2() -> u64 {
    let elr: u64;
    unsafe {
        core::arch::asm!("mrs {}, elr_el2", out(reg) elr);
    }
    elr
}

#[inline]
pub fn read_spsr_el2() -> u64 {
    let spsr: u64;
    unsafe {
        core::arch::asm!("mrs {}, spsr_el2", out(reg) spsr);
    }
    spsr
}

#[inline]
fn configure_hcr_el2() {
    HCR_EL2.write(
        HCR_EL2::VM::Enable
            + HCR_EL2::RW::EL1AArch64
            + HCR_EL2::AMO::EL2Handled
            + HCR_EL2::IMO::EL2Handled
            + HCR_EL2::FMO::EL2Handled
    );
}

#[inline]
fn configure_sctlr_el2() {
    SCTLR_EL2.write(
        SCTLR_EL2::M::Enable
            + SCTLR_EL2::C::Cacheable
            + SCTLR_EL2::I::Cacheable
    );
}

#[inline]
fn configure_vector_table(vector_base: usize) {
    unsafe {
        core::arch::asm!(
            "msr vbar_el2, {}",
            in(reg) vector_base as u64,
            options(nostack)
        );
    }
}

#[inline]
fn configure_timer_el2() {
    // CNTHCTL_EL2: 控制对定时器寄存器的访问
    // Bit 0: EL1PCTEN (不 trap 物理计数访问)
    // Bit 1: EL1PCEN (不 trap 物理定时器访问)
    let cnthctl: u64 = 0x3;
    unsafe {
        core::arch::asm!("msr CNTHCTL_EL2, {}", in(reg) cnthctl);
    }
    
    // CNTVOFF_EL2: 虚拟定时器偏移
    let cntvoff: u64 = 0;
    unsafe {
        core::arch::asm!("msr CNTVOFF_EL2, {}", in(reg) cntvoff);
    }
}

/// Hypervisor 初始化
pub fn hyp_init() {
    // 配置 HCR_EL2
    configure_hcr_el2();
    
    // 配置 SCTLR_EL2
    configure_sctlr_el2();

    // 配置 EL2 定时器控制
    configure_timer_el2();

    // 内存屏障
    unsafe {
        core::arch::asm!("dsb sy", options(nostack));
        core::arch::asm!("isb sy", options(nostack));
    }
    
    // 获取向量表地址
    let vector_base = vector::get_vector_table_addr();
    configure_vector_table(vector_base);

    print_hcr_el2_info();
}

pub fn print_hcr_el2_info() {
    let hcr = read_hcr_el2();
    
    info!("HCR_EL2 information:");
    info!("  Base value: {:#018x}", hcr);
    info!("  VM: {}", (hcr & 1) != 0);
    info!("  RW: {}", (hcr & (1 << 31)) != 0);
    info!("  AMO: {}", (hcr & (1 << 3)) != 0);
    info!("  IMO: {}", (hcr & (1 << 4)) != 0);
    info!("  FMO: {}", (hcr & (1 << 5)) != 0);
}

/// 从 EL2 进入 EL1 (Guest)
#[inline]
pub fn enter_guest(entry: usize, stack_top: usize, pstate: u64) {
    unsafe {
        // 设置 SP_EL1
        core::arch::asm!("msr sp_el1, {}", in(reg) stack_top);
        
        // 设置 ELR_EL2 (返回地址)
        core::arch::asm!("msr elr_el2, {}", in(reg) entry as u64);
        
        // 设置 SPSR_EL2 (目标 PSTATE)
        core::arch::asm!("msr spsr_el2, {}", in(reg) pstate);
        
        core::arch::asm!("dsb sy", options(nostack));
        core::arch::asm!("isb sy", options(nostack));
        
        // 切换
        core::arch::asm!("eret");
    }
}