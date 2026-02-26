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
        // ==========================================
        // 0. Ensure Alignment (Host & Guest)
        // ==========================================
        ".p2align 12", // 4KB align start of guest code
        
        // FORCE FLUSH I-CACHE to ensure we see new instructions
        "ic iallu",
        "isb",

        // ==========================================
        // 1. Entry Point
        // ==========================================
        "movz x0, #0xBEEF",
        "movk x0, #0xDEAD, lsl #16",
        "hvc #0", // Should print 0xDEADBEEF
        
        "movz x0, {stack_top_lo}",
        "movk x0, {stack_top_hi}, lsl #16",
        "mov sp, x0",
        "b 4f",

        // ==========================================
        // 2. Vector Table (Must be 2KB aligned)
        // ==========================================
        ".p2align 11",
        "1:",
        
        // ------------------------------------------
        // Current EL with SP0 (Offset 0x000 - 0x180)
        // ------------------------------------------
        // 0x000: Sync
        "b .", 
        ".p2align 7", 
        // 0x080: IRQ
        "b .", 
        ".p2align 7",
        // 0x100: FIQ
        "b .", 
        ".p2align 7",
        // 0x180: SError
        "b .", 
        ".p2align 7",
        
        // ------------------------------------------
        // Current EL with SPx (Offset 0x200 - 0x380)
        // ------------------------------------------
        // 0x200: Sync
        "b 3f",       
        ".p2align 7",
        // 0x280: IRQ
        "b 2f",  
        ".p2align 7",
        // 0x300: FIQ
        "b .",                        
        ".p2align 7",
        // 0x380: SError
        "b .",                        
        ".p2align 7",
        
        // ------------------------------------------
        // Lower EL using AArch64 (Offset 0x400 - 0x580)
        // ------------------------------------------
        // 0x400: Sync
        "b .", 
        ".p2align 7",
        // 0x480: IRQ
        "b .", 
        ".p2align 7",
        // 0x500: FIQ
        "b .", 
        ".p2align 7",
        // 0x580: SError
        "b .", 
        ".p2align 7",

        // ------------------------------------------
        // Lower EL using AArch32 (Offset 0x600 - 0x780)
        // ------------------------------------------
        // 0x600: Sync
        "b .", 
        ".p2align 7",
        // 0x680: IRQ
        "b .", 
        ".p2align 7",
        // 0x700: FIQ
        "b .", 
        ".p2align 7",
        // 0x780: SError
        "b .", 
        ".p2align 7",

        // ==========================================
        // 3. Handlers & Main Logic
        // ==========================================
        "2:",
        // FIX: Save context FIRST before clobbering x0/x1/x2
        "sub sp, sp, #32",
        "stp x0, x1, [sp, #16]",
        "stp x2, x3, [sp]", // Save x2 as we use it for timer control

        // Debug removed to prevent nested HVC side-effects
        
        "mrs x0, S3_0_C12_C12_0", // IAR
        "and x0, x0, #0xffffff",
        "cmp x0, #1020",
        "b.ge 1f",
        
        // ============================================
        // 1. Clear Interrupt Source (Reset Timer)
        // ============================================
        // Disable Timer completely to verify main loop execution
        "mov x2, #0",
        "msr CNTV_CTL_EL0, x2",
        
        "isb", // Ensure timer update is visible

        // ============================================
        // 2. EOI (Deactivate Interrupt)
        // ============================================
        "1:",
        "msr S3_0_C12_C12_1, x0", // EOIR
        
        "ldp x2, x3, [sp]",
        "ldp x0, x1, [sp, #16]",
        "add sp, sp, #32",
        "eret",

        "3:",
        // Simple Sync Handler: Check ESR and maybe HVC
        "mrs x0, esr_el1",
        "lsr x1, x0, #26", // EC
        "cmp x1, #0x24",   // Data Abort
        "b.eq 2f",
        "cmp x1, #0x20",   // Instruction Abort
        "b.eq 2f",
        "b 3f",
        "2:",
        "hvc #1", // Report Fault
        "3:",
        "eret",

        // ==========================================
        // 3. Main Function Logic
        // ==========================================
        "4:",
        // Debug: Entry check
        "hvc #1",

        // Set VBAR dynamically using ADR
        "adr x1, 1b",
        
        // Debug: Print VBAR address to verify alignment
        "mov x0, x1", 
        "hvc #0", 
        
        "msr vbar_el1, x1",
        "isb",
        
        // Enable SRE_EL1",
        "tbnz x0, #0, 4f", // If bit 0 is 1, skip
        "orr x0, x0, #1",
        "msr ICC_SRE_EL1, x0",
        "4:",

        // Enable Group 1
        "mov x0, #1",
        "msr ICC_IGRPEN1_EL1, x0",

        // Set PMR
        "mov x0, #0xFF",
        "msr ICC_PMR_EL1, x0",

        // Enable IRQ in PSTATE
        "msr daifclr, #2",

        // Inject IRQ (HVC #3) - Optional now, but keeps VBAR fix
        "hvc #3",

        // Simplified Main Loop
        "9:",
        // "wfi", // REMOVED to avoid potential side effects
        "nop",
        "mov x0, #0x1234",
        "hvc #0", // Heartbeat
        
        "nop",
        "nop",
        "10:",
        // "mov x0, #0x5678", 
        "add x0, x0, #1",     // Try to modify x0 based on previous value to check ALU
        "hvc #0",
        
        "add x0, x0, #1",
        "hvc #0",
        
        // Loop with WFI
        "11:",
        "wfi",
        "b 11b",

        stack_top_lo = const GUEST_STACK_TOP_LO,
        stack_top_hi = const GUEST_STACK_TOP_HI,
    );
}

// Remove all other functions to clean up
// #[no_mangle] pub unsafe extern "C" fn guest_vector_table() ...
// #[no_mangle] pub unsafe extern "C" fn guest_main() ...

// All other functions merged into guest_entry

/// Get guest code address.
#[inline]
pub fn guest_code_address() -> usize {
    guest_entry as *const () as usize
}

/// Get guest stack top address.
#[inline]
pub fn guest_stack_top() -> usize {
    GUEST_STACK_TOP
}