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

pub const GUEST_CODE_LOAD_ADDR: usize = 0x4100_0000;
pub const GUEST_STACK_SIZE: usize = 32 * 1024;
pub const GUEST_STACK_TOP: usize = 0x4110_0000 - 16;
const GUEST_STACK_TOP_LO: u16 = (GUEST_STACK_TOP & 0xFFFF) as u16;
const GUEST_STACK_TOP_HI: u16 = ((GUEST_STACK_TOP >> 16) & 0xFFFF) as u16;
pub const LINUX_KERNEL_LOAD_ADDR: usize = 0x4028_0000;
pub const LINUX_DTB_ADDR: usize = 0x4180_0000;
pub const LINUX_IMAGE: &[u8] = include_bytes!("../../../../../../Image");
pub const LINUX_DTB: &[u8] = include_bytes!("../../../../../../guest.dtb");

// #[no_mangle]
// #[naked]
// pub unsafe extern "C" fn guest_entry() -> ! {
//     core::arch::naked_asm!(
//         ".p2align 12", 
        
//         "movz x0, {stack_top_lo}",
//         "movk x0, {stack_top_hi}, lsl #16",
//         "mov sp, x0",

//         "adr x0, 100f", 
//         "msr vbar_el1, x0",
//         "isb",

//         "movz x0, #0xBEEF",
//         "movk x0, #0xDEAD, lsl #16",
//         "hvc #0x10",
//         "nop",
        
//         "b 500f",

//         // ==========================================
//         // Vector Table (Strict 2KB alignment is required.)
//         // ==========================================
//         ".p2align 11",
//         "100:",
//         "b .",  ".p2align 7", "b .",  ".p2align 7", "b .",  ".p2align 7", "b .",  ".p2align 7",
//         "b 300f",  ".p2align 7",    // Sync
//         "b 200f",  ".p2align 7",    // IRQ
//         "b .",     ".p2align 7", "b .",  ".p2align 7",
//         "b .",  ".p2align 7", "b .",  ".p2align 7", "b .",  ".p2align 7", "b .",  ".p2align 7",
//         "b .",  ".p2align 7", "b .",  ".p2align 7", "b .",  ".p2align 7", "b .",  ".p2align 7",

//         // ==========================================
//         // IRQ Handler
//         // ==========================================
//         "200:",
//         "sub sp, sp, #48",
//         "stp x0, x1, [sp, #0]",
//         "stp x2, x3, [sp, #16]",
//         "stp x4, x5, [sp, #32]",

//         "mov x0, #0xB1",
//         "hvc #0x10",
//         "nop",

//         "mrs x0, ICC_IAR1_EL1", 
//         "and x0, x0, #0xFFFFFF", 

//         "msr ICC_EOIR1_EL1, x0",  
//         "isb",
        
//         "mov x19, #0",
//         "mov x0, #0xB2",
//         "hvc #0x10",
//         "nop",

//         "hvc #0x15",
//         "nop",

//         "ldp x4, x5, [sp, #32]",
//         "ldp x2, x3, [sp, #16]",
//         "ldp x0, x1, [sp, #0]",
//         "add sp, sp, #48",
//         "eret",

//         // ==========================================
//         // Sync Handler
//         // ==========================================
//         "300:",
//         "hvc #1", 
//         "nop",
//         "eret",

//         // ==========================================
//         // 🚀 Main Logic
//         // ==========================================
//         "500:",
//         "mrs x0, ICC_SRE_EL1",
//         "orr x0, x0, #1",
//         "msr ICC_SRE_EL1, x0",
//         "isb",

//         "mov x0, #1",
//         "msr ICC_IGRPEN1_EL1, x0",
//         "mov x0, #0xFF",
//         "msr ICC_PMR_EL1, x0",
//         "msr daifclr, #2",
//         "isb",

//         // [test 1] access normal memory
//         "movz x5, #0x0000",
//         "movk x5, #0x4120, lsl #16",    
//         "movz x6, #0xABCD",
//         "movk x6, #0x1234, lsl #16",    
//         "str x6, [x5]",                  
//         "dsb sy",
//         "isb",
//         "ldr x7, [x5]",                  
//         "cmp x6, x7",
//         "b.ne 503f",                     
//         "mov x0, #0x51",
//         "hvc #0x10",
//         "nop",
//         "b 504f",
//         "503:",
//         "mov x0, #0x5F",
//         "hvc #0x10",                     
//         "nop",
        
//         "504:",
//         // [test 2] read vGIC MMIO
//         "movz x5, #0x0008",
//         "movk x5, #0x0800, lsl #16",    
//         "ldr w6, [x5]",                 
//         "mov x0, x6",
//         "hvc #0x10",                    
//         "nop",

//         // [test 3] write vGIC MMIO
//         "movz x5, #0x0000",
//         "movk x5, #0x0800, lsl #16",
//         "mov w6, #1",
//         "str w6, [x5]",              

//         "movz x5, #0x0104",
//         "movk x5, #0x0800, lsl #16",
//         "mov w6, #1",
//         "str w6, [x5]",           

//         "mov x0, #0xA1",
//         "hvc #0x10",
//         "nop",
//         // [test 4] irq inject
//         "mov x19, #1",
//         "hvc #0x13", 
//         "nop",

//         "510:",
//         "cbnz x19, 510b",

//         "mov x0, #0xA2",    
//         "hvc #0x10",
//         "nop",
        
//         "mov x0, #0x49",
//         "hvc #0x10",
//         "nop",

//         "hvc #0x20",
//         "nop",
//         "b .",

//         stack_top_lo = const GUEST_STACK_TOP_LO,
//         stack_top_hi = const GUEST_STACK_TOP_HI,
//     );
// }

// #[inline]
// pub fn guest_code_address() -> usize {
//     guest_entry as *const () as usize
// }

/// Get guest stack top address.
#[inline]
pub fn guest_stack_top() -> usize {
    GUEST_STACK_TOP
}