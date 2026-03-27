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

pub const GUEST_CODE_LOAD_ADDR: usize = 0x4800_0000;
pub const GUEST_STACK_SIZE: usize = 32 * 1024;
pub const GUEST_STACK_TOP: usize = 0x4810_0000 - 16;
const GUEST_STACK_TOP_LO: u16 = (GUEST_STACK_TOP & 0xFFFF) as u16;
const GUEST_STACK_TOP_HI: u16 = ((GUEST_STACK_TOP >> 16) & 0xFFFF) as u16;

#[no_mangle]
#[naked]
pub unsafe extern "C" fn guest_entry() -> ! {
    core::arch::naked_asm!(
        ".p2align 12", // 4KB align start of guest code
        
        // FORCE FLUSH I-CACHE to ensure we see new instructions
        // "ic iallu",
        // "isb",

        // ==========================================
        // 1. Entry Point
        // ==========================================
        "movz x0, #0xBEEF",
        "movk x0, #0xDEAD, lsl #16",
        "hvc #0x10", // Hello
        
        "movz x0, {stack_top_lo}",
        "movk x0, {stack_top_hi}, lsl #16",
        "mov sp, x0",
        "b 500f",

        // ==========================================
        // 2. Vector Table (Must be 2KB aligned)
        // ==========================================
        ".p2align 11",
        "100:",
        // Current EL with SP0
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",

        // Current EL with SPx
        "b 300f",  ".p2align 7",    // 0x200 Sync
        "b 200f",  ".p2align 7",    // 0x280 IRQ
        "b .",     ".p2align 7",    // 0x300 FIQ
        "b .",     ".p2align 7",    // 0x380 SError

        // Lower EL AArch64
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",

        // Lower EL AArch32
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",
        "b .",  ".p2align 7",

        // ==========================================
        // IRQ Handler 
        // ==========================================
        "200:",
        "sub sp, sp, #48",
        "stp x0, x1, [sp, #0]",
        "stp x2, x3, [sp, #16]",
        "stp x4, x5, [sp, #32]",

        "mrs x0, S3_0_C12_C12_0", 
        "and x0, x0, #0xFFFFFF",
        "mov x1, #1020",
        "cmp x0, x1",
        "b.ge 201f",

        // Close vtimer
        "mov x2, #0",
        "msr CNTV_CTL_EL0, x2",
        "isb",

        "201:",
        "msr S3_0_C12_C12_1, x0",  // ICC_EOIR1_EL1: EOI
        "isb",
        "hvc #0x15",

        "ldp x4, x5, [sp, #32]",
        "ldp x2, x3, [sp, #16]",
        "ldp x0, x1, [sp, #0]",
        "add sp, sp, #48",
        "eret",

        // ==========================================
        // Sync Handler (offset 0x200)
        // ==========================================
        "300:",
        "mrs x0, esr_el1",
        "lsr x1, x0, #26",
        "cmp x1, #0x24",
        "b.eq 301f",
        "cmp x1, #0x20",
        "b.eq 301f",
        "eret",
        "301:",
        "hvc #1",
        "eret",

        // ==========================================
        // Main
        // ==========================================
        "500:",
        // Set VBAR_EL1
        "adr x0, 100b",
        "msr vbar_el1, x0",
        "isb",

        // Config Sys register of GIC.
        "mrs x0, ICC_SRE_EL1",
        "orr x0, x0, #1",
        "msr ICC_SRE_EL1, x0",
        "isb",

        // Enable Group1
        "mov x0, #1",
        "msr ICC_IGRPEN1_EL1, x0",

        // Set priority mask
        "mov x0, #0xFF",
        "msr ICC_PMR_EL1, x0",

        // Open IRQ
        "msr daifclr, #2",
        "isb",

        // ------------------------------------------
        // [STEP 2] MMU Stage-2 test
        // ------------------------------------------
        "movz x5, #0x0000",
        "movk x5, #0x4780, lsl #16",    // x5 = 0x4780_0000（已映射，远离代码/栈）
        "movz x6, #0xABCD",
        "movk x6, #0x1234, lsl #16",    // x6 = 0x1234_ABCD（魔数）
        "str x6, [x5]",                  // 写入
        "dsb sy",
        "isb",
        "ldr x7, [x5]",                  // 读回
        "cmp x6, x7",
        "b.ne 503f",                     // 不一致 → 失败
        "mov x0, #0x51",
        "hvc #0x10",                     // 上报：[STEP 2a] 映射地址读写 OK
        "b 504f",

        "503:",
        "mov x0, #0x5F",                 // 失败码
        "hvc #0x10",                     // 上报：[STEP 2a] 读写不一致，FAIL

        // Test 2b: 访问未映射地址，期望触发 Stage-2 Fault 并被 EL2 跳过
        "504:",
        "movz x0, #0x0000",
        "movk x0, #0x5000, lsl #16",    // 0x5000_0000（未映射）
        "ldr x1, [x0]",                  // 触发 Stage-2 Translation Fault，EL2 跳过此指令
        "mov x0, #0x52",
        "hvc #0x10",

        // ------------------------------------------
        // [STEP 3] IRQ inject to test vGIC
        // ------------------------------------------
        "nop",
        "nop",
        "hvc #0x13",
        "nop",
        "mov x2, #200",
        "510:",
        "nop",
        "nop",
        "sub x2, x2, #1",
        "cbnz x2, 510b",

        "mov x0, #0x49",
        "hvc #0x10", 

        // ------------------------------------------
        // [STEP 4] Shutdown
        // ------------------------------------------
        "nop",
        "nop",
        "hvc #0x20",

        // Shouldn't reach here.
        "b .",

        stack_top_lo = const GUEST_STACK_TOP_LO,
        stack_top_hi = const GUEST_STACK_TOP_HI,
    );
}


#[inline]
pub fn guest_code_address() -> usize {
    guest_entry as *const () as usize
}

/// Get guest stack top address.
#[inline]
pub fn guest_stack_top() -> usize {
    GUEST_STACK_TOP
}