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

use super::ProcFileOps;
use crate::{
    error::Error,
    irq::irq_trace::{IrqTraceInfo, IRQ_COUNTERS, PER_CPU_TRACE_INFO},
    scheduler, thread, time,
};
use alloc::{string::String, vec::Vec};
use core::{fmt::Write, sync::atomic::Ordering::Relaxed};

pub(crate) struct SystemStat;

const NUM_CORES: usize = blueos_kconfig::CONFIG_NUM_CORES as usize;

// Change: using one string to recieve and delete Display.
impl ProcFileOps for SystemStat {
    fn get_content(&self) -> Result<Vec<u8>, Error> {
        #[cfg(debug_assertions)]
        {
            // CPU part: ~100 bytes per line * (total + every cpu core)
            // IRQ part: ~16 bytes per counter (conservative estimate)
            let capacity = 100 * (NUM_CORES + 1) + IRQ_COUNTERS.len() * 16;
            let mut result = String::with_capacity(capacity);
            append_cpu_time(&mut result);

            result.push('\n');
            append_irq_counts(&mut result);
            Ok(result.into_bytes())
        }

        #[cfg(not(debug_assertions))]
        {
            Ok("Skip in release".as_bytes().to_vec())
        }
    }
    fn set_content(&self, _content: Vec<u8>) -> Result<usize, Error> {
        Ok(0)
    }
}
#[derive(Default, Copy, Clone)]
struct CpuStat {
    cpu_id: usize,
    user: u64,
    nice: u64,
    system: u64,
    idle: u64,
    iowait: u64,
    irq: u64,
    softirq: u64,
    steal: u64,
    guest: u64,
    guest_nice: u64,
}

fn append_cpu_time(result: &mut String) {
    let mut total_system_time: u64 = 0;
    let mut total_idle_time: u64 = 0;
    let mut total_irq_time: u64 = 0;
    let mut cpu_stats = [CpuStat::default(); NUM_CORES + 1];
    loop {
        let pg = thread::Thread::try_preempt_me();
        if !pg.preemptable() {
            continue;
        }
        let total_cycle: u64 = time::get_sys_cycles();
        for cpu_id in 0..NUM_CORES {
            let idle_thread = scheduler::get_idle_thread(cpu_id);
            let idle_cycle = if idle_thread.state() == thread::RUNNING {
                idle_thread.get_cycles() + total_cycle - idle_thread.start_cycles()
            } else {
                idle_thread.get_cycles()
            };
            let system_time = time::cycles_to_millis(total_cycle.saturating_sub(idle_cycle)) / 10; // 10ms
            let idle_time = time::cycles_to_millis(idle_cycle) / 10;
            let irq_trace: &IrqTraceInfo = unsafe { &PER_CPU_TRACE_INFO[cpu_id] };
            let irq_time = time::cycles_to_millis(irq_trace.total_irq_process_cycles) / 10;
            total_system_time += system_time;
            total_idle_time += idle_time;
            total_irq_time += irq_time;

            let stat = &mut cpu_stats[cpu_id + 1];
            stat.cpu_id = cpu_id;
            stat.system = system_time;
            stat.idle = idle_time;
            stat.irq = irq_time;
        }
        let total_stat = &mut cpu_stats[0];
        total_stat.cpu_id = NUM_CORES; // total
        total_stat.system = total_system_time;
        total_stat.idle = total_idle_time;
        total_stat.irq = total_irq_time;
        break;
    }

    for stat in &cpu_stats {
        if stat.cpu_id == NUM_CORES {
            writeln!(
                result,
                "cpu  {} {} {} {} {} {} {} {} {} {}",
                stat.user,
                stat.nice,
                stat.system,
                stat.idle,
                stat.iowait,
                stat.irq,
                stat.softirq,
                stat.steal,
                stat.guest,
                stat.guest_nice
            )
            .unwrap();
        } else {
            writeln!(
                result,
                "cpu{} {} {} {} {} {} {} {} {} {} {}",
                stat.cpu_id,
                stat.user,
                stat.nice,
                stat.system,
                stat.idle,
                stat.iowait,
                stat.irq,
                stat.softirq,
                stat.steal,
                stat.guest,
                stat.guest_nice
            )
            .unwrap();
        }
    }
}

fn append_irq_counts(result: &mut String) {
    let mut total_count: u64 = 0;

    for atomic in &IRQ_COUNTERS {
        let count = atomic.load(Relaxed) as u64;
        total_count = total_count.saturating_add(count);
    }

    const PREFIX: &str = "intr ";
    write!(result, "{} {}", PREFIX, total_count).unwrap();

    for element in &IRQ_COUNTERS {
        let count = element.load(Relaxed);
        write!(result, " {}", count).unwrap();
    }
}
