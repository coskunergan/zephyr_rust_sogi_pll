// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use zephyr::kconfig::CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
use zephyr::raw::k_cycle_get_32;

static mut LAST_CYCLES: u32 = 0;
const CLOCK_FREQ: u64 = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC as u64;
#[allow(dead_code)]
pub fn get_cycle_count() -> u32 {
    unsafe { k_cycle_get_32() }
}
#[allow(dead_code)]
pub fn set_last_cycles(value: u32) {
    unsafe {
        LAST_CYCLES = value;
    }
}
#[allow(dead_code)]
pub fn get_last_cycles() -> u32 {
    unsafe { LAST_CYCLES }
}
#[allow(dead_code)]
pub fn set_logger() -> Result<(), &'static str> {
    unsafe { zephyr::set_logger() }.map_err(|_| "Logger failure.")
}
#[allow(dead_code)]
pub fn cycles_to_microseconds(cycles: u32) -> u64 {
    (cycles as u64 * 1_000_000) / CLOCK_FREQ
}
#[allow(dead_code)]
pub fn cycles_to_nanoseconds(cycles: u32) -> u64 {
    (cycles as u64 * 1_000_000_000) / CLOCK_FREQ
}
#[allow(dead_code)]
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
#[allow(dead_code)]
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
