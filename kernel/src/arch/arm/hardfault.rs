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

use super::{xpsr, IsrContext};
use core::fmt;
use cortex_m::peripheral::SCB;

#[derive(Debug, Default)]
struct HardFaultRegs {
    cfsr: u32,  // Configurable Fault Status Register
    hfsr: u32,  // Hard Fault Status Register
    mmfar: u32, // Memory Management Fault Address Register
    bfar: u32,  // Bus Fault Address Register
    afsr: u32,  // Auxiliary Fault Status Register (ARMv8-M)
}

impl HardFaultRegs {
    pub fn from_scb() -> Self {
        // Get the value of the SCB registers
        // SAFETY: SCB::PTR comes from cortex_m crate and is a valid pointer
        let scb = unsafe { &*SCB::PTR };

        Self {
            cfsr: scb.cfsr.read(),
            hfsr: scb.hfsr.read(),
            mmfar: scb.mmfar.read(),
            bfar: scb.bfar.read(),
            afsr: scb.afsr.read(),
        }
    }
}

impl fmt::Display for HardFaultRegs {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        #[cfg(debug_assertions)]
        {
            // Define a marco to check digit
            // Using write_str to avoid the spending of  constructing format_args!().
            macro_rules! check {
                ($reg:expr, $bit:expr, $msg:expr) => {
                    if $reg & (1 << $bit) != 0 {
                        f.write_str("    - ")?;
                        f.write_str($msg)?;
                        f.write_str("\n")?;
                    }
                };
            }
            writeln!(f, "\nHFSR: 0x{:08x}", self.hfsr)?;
            if self.hfsr & (1 << 30) != 0 {
                f.write_str("  - Forced Hard Fault\n")?;
            }
            if self.hfsr & (1 << 31) != 0 {
                f.write_str("  - Debug Event\n")?;
            }
            if self.hfsr & (1 << 1) != 0 {
                f.write_str("  - Vector Table Read Fault\n")?;
            }
            writeln!(f, "CFSR: 0x{:08x}", self.cfsr)?;
            f.write_str("Fault Status:\n")?;
            // ---------------------------------------------------------------------
            // Memory Management Faults (Byte 0)
            // ---------------------------------------------------------------------
            if self.cfsr & 0xFF != 0 {
                // https://developer.arm.com/documentation/ddi0553/latest/
                // MMFARVALID, bit [7] - MMFAR valid flag. Indicates validity of the MMFAR register.
                //                       0: MMFAR content not valid.
                //                       1: MMFAR content valid
                // MLSPERR, bit [5]    - MemManage lazy Floating-point state preservation error flag.
                //                       Records whether a MemManage fault occurred during lazy Floating-point state preservation.
                //                       0: No MemManage occurred.
                //                       1: MemManage occurred.
                // MSTKERR, bit [4]    - MemManage stacking error flag. Records whether a derived MemManage fault occurred during exception entry stacking.
                //                       0: No derived MemManage occurred.
                //                       1: Derived MemManage occurred during exception entry.
                // MUNSTKERR, bit [3]  - MemManage unstacking error flag. Records whether a derived MemManage fault occurred during exception return unstacking.
                //                       0: No derived MemManage fault occurred.
                //                       1: Derived MemManage fault occurred during exception return
                // DACCVIOL, bit [1]   - Data access violation flag. Records whether a data access violation has occurred.
                //                       0: No MemManage fault on data access has occurred.
                //                       1:  MemManage fault on data access has occurred.
                //                       A DACCVIOL will be accompanied by an MMFAR update.
                // IACCVIOL, bit [0]   - Instruction access violation. Records whether an instruction related memory access violation has occurred.
                //                       0: No MemManage fault on instruction access has occurred.
                //                       1: MemManage fault on instruction access has occurred.

                f.write_str("  Memory Management Fault:\n")?;

                check!(self.cfsr, 0, "Instruction access violation");
                check!(self.cfsr, 1, "Data access violation");
                check!(self.cfsr, 3, "Unstacking error");
                check!(self.cfsr, 4, "Stacking error");
                check!(self.cfsr, 5, "lazy Floating-point state preservation error");
                if self.cfsr & (1 << 7) != 0 {
                    f.write_str("    - MMFAR valid\n")?;
                    writeln!(f, "      Fault Address: 0x{:08x}", self.mmfar)?;
                }
            }
            // ---------------------------------------------------------------------
            // Bus Faults (Byte 1)
            // ---------------------------------------------------------------------
            if self.cfsr & 0xFF00 != 0 {
                // BFARVALID, bit [7] - BFAR valid. Indicates validity of the contents of the BFAR register.
                //                      0: BFAR content not valid.
                //                      1: BFAR content valid.
                // LSPERR, bit [5]   - Lazy state preservation error. Records whether a precise BusFault occurred during floating-point lazy
                //                     Floating-point state preservation.
                //                      0: No BusFault occurred.
                //                      1: BusFault occurred.
                //                     If AIRCR.BFHFNMINS is zero this bit is RAZ/WI from Non-secure state.
                // STKERR, bit [4]   - Stack error. Records whether a precise derived BusFault occurred during exception entry stacking.
                //                      0: No derived BusFault occurred.
                //                      1: Derived BusFault occurred during exception entry.
                //                    Derived BusFault occurred during exception entry.
                //                    If AIRCR.BFHFNMINS is zero this bit is RAZ/WI from Non-secure state.
                // UNSTKERR, bit [3] - Unstack error. Records whether a precise derived BusFault occurred during exception return unstacking.
                //                      0 :No derived BusFault occurred.
                //                      1: Derived BusFault occurred during exception return.
                //                     If AIRCR.BFHFNMINS is zero this bit is RAZ/WI from Non-secure state.
                // IMPRECISERR, bit [2] - Imprecise error. Records whether an imprecise data access error has occurred.
                //                      0: No imprecise data access error has occurred.
                //                      1: Imprecise data access error has occurred.
                //                     If AIRCR.BFHFNMINS is zero this bit is RAZ/WI from Non-secure state.
                // PRECISERR, bit [1] - Precise error. Records whether a precise data access error has occurred.
                //                      0: No precise data access error has occurred.
                //                      1: Precise data access error has occurred.
                //                     When a precise error is recorded, the associated address is written to the BFAR and BFSR.BFARVALID bit
                //                     is set.
                //                     If AIRCR.BFHFNMINS is zero this bit is RAZ/WI from Non-secure state.
                // IBUSERR, bit [0] - Instruction bus error. Records whether a precise BusFault on an instruction prefetch has occurred.
                //                      0: No BusFault on instruction prefetch has occurred.
                //                      1: A BusFault on an instruction prefetch has occurred.
                //                     An IBUSERR is only recorded if the instruction is issued for execution.
                //                     If AIRCR.BFHFNMINS is zero this bit is RAZ/WI from Non-secure state.

                f.write_str("  Bus Fault:\n")?;
                check!(self.cfsr, 8, "Instruction bus error");
                check!(self.cfsr, 9, "Precise error");
                check!(self.cfsr, 10, "Imprecise error");
                check!(self.cfsr, 11, "Unstack error");
                check!(self.cfsr, 12, "Stacking error");
                check!(self.cfsr, 13, "Lazy state preservation error");
                if self.cfsr & (1 << 15) != 0 {
                    f.write_str("    - BFAR valid\n")?;
                    writeln!(f, "      Fault Address: 0x{:08x}", self.bfar)?;
                }
            }
            // ---------------------------------------------------------------------
            // Usage Faults (Upper Half)
            // ---------------------------------------------------------------------
            // DIVBYZERO, bit [9] - Divide by zero flag. Sticky flag indicating whether an integer division by zero error has occurred.
            //                      0: Error has not occurred.
            //                      1: Error has occurred.
            // UNALIGNED, bit [8] - Unaligned access flag. Sticky flag indicating whether an unaligned access error has occurred.
            //                      0: Error has not occurred.
            //                      1: Error has occurred.
            // Bits [7:5]         -  Reserved, RES0.
            // STKOF, bit [4]     - Stack overflow flag. Sticky flag indicating whether a stack overflow error has occurred.
            //                      0: Error has not occurred.
            //                      1: Error has occurred.
            // NOCP, bit [3]       - No coprocessor flag. Sticky flag indicating whether a coprocessor disabled or not present error has occurred.
            //                      0: Error has not occurred.
            //                      1: Error has occurred.
            // INVPC, bit [2]      - Invalid PC flag. Sticky flag indicating whether an integrity check error has occurred.
            //                      0: Error has not occurred.
            //                      1: Error has occurred.
            // INVSTATE, bit [1]    - Invalid state flag. Sticky flag indicating whether an EPSR.B, EPSR.T, EPSR.IT, or FPSCR.LTPSIZE validity
            //                      0: Error has not occurred.
            //                      1: Error has occurred.
            // UNDEFINSTR, bit [0]   - UNDEFINED instruction flag. Sticky flag indicating whether an UNDEFINED instruction error has occurred.
            //                      0: Error has not occurred.
            //                      1: Error has occurred.
            //                      This includes attempting to execute an UNDEFINED instruction associated with an enable coprocessor.

            if self.cfsr & 0xFFFF0000 != 0 {
                f.write_str("  Usage Fault:\n")?;
                check!(self.cfsr, 16, "Undefined instruction");
                check!(self.cfsr, 17, "Invalid state");
                check!(self.cfsr, 18, "Invalid PC load");
                check!(self.cfsr, 19, "No coprocessor");
                #[cfg(armv8m)]
                check!(self.cfsr, 20, "Stack overflow");
                check!(self.cfsr, 24, "Unaligned access");
                check!(self.cfsr, 25, "Divide by zero");
            }
            writeln!(f, "AFSR: 0x{:08x}", self.afsr)?;
            if self.afsr != 0 {
                f.write_str("  - Auxiliary Faults detected\n")?;
            }
            Ok(())
        }

        #[cfg(not(debug_assertions))]
        {
            // Keep basic log in release mode.
            write!(
                f, 
                "HFSR:{:x} CFSR:{:x} MMFAR:{:x} BFAR:{:x} AFSR:{:x}", 
                self.hfsr, self.cfsr, self.mmfar, self.bfar, self.afsr
            )
        }
    }
}

pub extern "C" fn panic_on_hardfault(ctx: &IsrContext) {
    super::disable_local_irq();
    let fault_regs: HardFaultRegs = HardFaultRegs::from_scb();
    let xpsr = xpsr::read();
    panic!(
        "
        ==== HARD FAULT ====
        FRAME: {:?}
        FAULT REGS: {}
        XPSR: {}
        ",
        ctx, fault_regs, xpsr,
    );
}

#[naked]
pub(crate) unsafe extern "C" fn handle_hardfault() {
    core::arch::naked_asm!(
        "
        mrs r0, msp
        tst lr, #0x04
        beq 1f
        mrs r0, psp
        1:
        bl {panic}
        ",
        panic = sym panic_on_hardfault
    )
}
