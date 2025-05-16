// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use zephyr::kconfig::CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
use zephyr::raw::k_cycle_get_32;

static mut LAST_CYCLES: u32 = 0;
const CLOCK_FREQ: u64 = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC as u64;

pub fn get_cycle_count() -> u32 {
    unsafe { k_cycle_get_32() }
}

pub fn set_last_cycles(value: u32) {
    unsafe {
        LAST_CYCLES = value;
    }
}

pub fn get_last_cycles() -> u32 {
    unsafe { LAST_CYCLES }
}

pub fn set_logger_safe() -> Result<(), &'static str> {
    unsafe { zephyr::set_logger() }.map_err(|_| "Logger ayarı başarısız")
}

pub fn cycles_to_microseconds(cycles: u32) -> u64 {
    (cycles as u64 * 1_000_000) / CLOCK_FREQ
}

pub fn cycles_to_nanoseconds(cycles: u32) -> u64 {
    (cycles as u64 * 1_000_000_000) / CLOCK_FREQ
}

pub fn measure_function_duration_us<F>(func: F) -> u64
where
    F: FnOnce(),
{
    let start = get_cycle_count();
    func();
    let end = get_cycle_count();
    let cycles = end.wrapping_sub(start);
    cycles_to_microseconds(cycles)
}

pub fn measure_function_duration_ns<F>(func: F) -> u64
where
    F: FnOnce(),
{
    let start = get_cycle_count();
    func();
    let end = get_cycle_count();
    let cycles = end.wrapping_sub(start);
    cycles_to_nanoseconds(cycles)
}
