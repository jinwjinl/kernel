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

use crate::{allocator, error::Error, vfs::procfs::ProcFileOps};
use alloc::{format, string::String, vec::Vec};
use core::fmt::Write;

pub(crate) struct MemoryInfo;

impl ProcFileOps for MemoryInfo {
    fn get_content(&self) -> Result<Vec<u8>, Error> {
        let meminfo = allocator::memory_info();
        let total = meminfo.total / 1024;
        let available = (meminfo.total - meminfo.used) / 1024;
        let used = meminfo.used / 1024;
        let max_used = meminfo.max_used / 1024;
        let mut result = String::with_capacity(128);

        let mut write_line = |label: &str, value: usize, width: usize| {
            let _ = write!(result, "{:<14}", label);
            let mut n = value;
            let mut len = 1;
            while n >= 10 {
                n /= 10;
                len += 1;
            }

            if width > len {
                for _ in 0..(width - len) {
                    let _ = result.write_char(' ');
                }
            }

            let _ = writeln!(result, "{} kB", value);
        };
        let mut write_aligned = |label_with_padding: &str, value: usize| {
            let _ = result.write_str(label_with_padding);
            let mut n = value;
            let mut len = 1;

            while n >= 10 {
                n /= 10;
                len += 1;
            }

            const TARGET_WIDTH: usize = 8;
            if TARGET_WIDTH > len {
                for _ in 0..(TARGET_WIDTH - len) {
                    let _ = result.write_char(' ');
                }
            }

            let _ = writeln!(result, "{} kB", value);
        };

        write_aligned("MemTotal:     ", total);
        write_aligned("MemAvailable: ", available);
        write_aligned("MemUsed:      ", used);
        write_aligned("MemMaxUsed:   ", max_used);
        Ok(result.into_bytes())
    }

    fn set_content(&self, content: Vec<u8>) -> Result<usize, Error> {
        Ok(0)
    }
}
