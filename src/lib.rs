// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#![no_std]

extern crate alloc;

use alloc::format;
use core::convert::TryInto;
use embassy_time::{Duration, Timer};

#[cfg(feature = "executor-thread")]
use embassy_executor::Executor;

#[cfg(feature = "executor-zephyr")]
use zephyr::embassy::Executor;

use core::{sync::atomic::AtomicBool, sync::atomic::AtomicI32, sync::atomic::Ordering};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use spin::Once;
use static_cell::StaticCell;
use zephyr::sync::{Arc, Mutex};

use adc_io::Adc;
use dac_io::Dac;
use display_io::Display;

mod adc_io;
mod dac_io;
mod display_io;
mod usage;

// Q15 Sabitler
const MAX_Q15: u32 = 4095; // DAC max
const MIN_Q15: u32 = 0; // DAC min

// SOGI-PLL durum yapısı
struct SogiPllState {
    v_alpha: i16,         // Q15
    v_beta: i16,          // Q15
    v_mid: i16,           // Q15
    omega: i32,           // Q15 (2π * 50 Hz)
    theta: u16,           // 0-65535 (2π)
    integral: i32,        // Q15
    auto_offset_min: u32, // 0..4095
    auto_offset_max: u32, // 0..4095
}

impl SogiPllState {
    const fn new() -> Self {
        SogiPllState {
            v_alpha: 0,
            v_beta: 0,
            v_mid: 0,
            omega: 31416, // 2π * 50 Hz, Q15
            theta: 0,
            integral: 0,
            auto_offset_min: MAX_Q15 << 4,
            auto_offset_max: MIN_Q15 << 4,
        }
    }
}

// Statik SOGI-PLL durumu (Mutex ile Sync)
static SOGI_STATE: StaticCell<Mutex<SogiPllState>> = StaticCell::new();
static SOGI_STATE_REF: Once<&'static Mutex<SogiPllState>> = Once::new();

// Sinüs lookup table (256 eleman, Q15)
const SIN_LUT: [i16; 256] = [
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
    2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
    3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
    4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
    3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
    2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
    2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697, 1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311,
    1264, 1218, 1172, 1127, 1082, 1038, 995, 952, 910, 869, 828, 788, 749, 710, 672, 636, 600, 565,
    530, 497, 465, 433, 403, 373, 345, 318, 291, 266, 242, 219, 197, 176, 156, 137, 120, 103, 88,
    74, 61, 50, 39, 30, 22, 15, 10, 6, 2, 1, 0, 1, 2, 6, 10, 15, 22, 30, 39, 50, 61, 74, 88, 103,
    120, 137, 156, 176, 197, 219, 242, 266, 291, 318, 345, 373, 403, 433, 465, 497, 530, 565, 600,
    636, 672, 710, 749, 788, 828, 869, 910, 952, 995, 1038, 1082, 1127, 1172, 1218, 1264, 1311,
    1358, 1405, 1453, 1501, 1550, 1599, 1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997,
];

// SOGI bloğu (Q15)
fn sogi_process_q15(
    input: i16,
    omega: i32,
    k: i16,
    ts: i16,
    state: &mut SogiPllState,
) -> (i16, i16) {
    let error = input - state.v_alpha;
    let delta_alpha = (((omega as i32 * state.v_beta as i32) >> 15)
        + ((k as i32 * error as i32) >> 15))
        * ts as i32
        >> 15;
    state.v_alpha += TryInto::<i16>::try_into(delta_alpha).unwrap();
    let delta_beta = ((-omega as i32 * state.v_alpha as i32) >> 15) * ts as i32 >> 15;
    state.v_beta += TryInto::<i16>::try_into(delta_beta).unwrap();
    (state.v_alpha, state.v_beta)
}

// Yaklaşık faz hatası hesaplama
fn approx_phase_error(v_alpha: i16, v_beta: i16) -> i16 {
    if v_alpha == 0 {
        0
    } else {
        match (((v_beta as i32) << 15) / v_alpha as i32).try_into() {
            Ok(val) => val,
            Err(_) => -1,
        }
    }
}

// PI denetleyici (Q15)
fn pi_controller_q15(error: i16, kp: i16, ki: i16, ts: i16, state: &mut SogiPllState) -> i16 {
    state.integral += ((ki as i32 * error as i32) >> 15) * ts as i32;
    state.integral = state.integral.clamp(-1 << 30, 1 << 30); // ±2^30
    TryInto::<i16>::try_into(((kp as i32 * error as i32) >> 15) + (state.integral >> 15)).unwrap()
}

// ADC geri çağrı fonksiyonu
fn adc_callback(idx: usize, value: i16) {
    // Sabitler
    const K: i16 = 4634; // 1.414 * 32768 / 2^15
    const TS: i16 = 328; // 100 µs (10 kHz), Q15
    const KP: i16 = 819; // 0.1 * 32768
    const KI: i16 = 82; // 0.01 * 32768
    const NOMINAL_OMEGA: i32 = 31416; // 2π * 50 Hz, Q15

    // Performans ölçümü: Başlangıç
    let start = usage::get_cycle_count();

    if idx == 0 {
        // SOGI-PLL durumuna güvenli erişim
        if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
            // Otomatik ofset
            let v_org = {
                state.auto_offset_max = state.auto_offset_max.saturating_sub(1);
                state.auto_offset_min = state.auto_offset_min.saturating_add(1);
                let offset_value: u32 = ((value as u32) << 4).try_into().unwrap(); //16
                if offset_value > state.auto_offset_max {
                    state.auto_offset_max = offset_value;
                }
                if offset_value < state.auto_offset_min {
                    state.auto_offset_min = offset_value;
                }
                let mid = (state.auto_offset_min as i32 + state.auto_offset_max as i32) >> 5;
                state.v_mid = mid as i16;
                (value as i32 - mid).clamp(-32768, 32767) as i16
                
            };

            // SOGI bloğu
            let (v_alpha, v_beta) = sogi_process_q15(v_org, state.omega, K, TS, &mut state);

            // Faz hatası
            let phase_error = approx_phase_error(v_alpha, v_beta).clamp(-36, 36) as i16;

            // PI denetleyici
            let omega_delta = pi_controller_q15(phase_error, KP, KI, TS, &mut state);

            // Omega’yı güvenli şekilde güncelle
            let new_omega = (NOMINAL_OMEGA as i32 + omega_delta as i32).clamp(25132, 37700) as i32;
            state.omega = new_omega;

            // Faz açısını güncelle
            state.theta = state
                .theta
                .wrapping_add(((state.omega as i32 * TS as i32) >> 15) as u16);

            // DAC çıkışı
            if let Some(dac) = DAC.get() {
                let sin_value = SIN_LUT[(state.theta >> 8) as usize] as u32;
                dac.write(sin_value);
            }
        }
    }

    // Performans ölçümü: Bitiş ve saklama
    let end = usage::get_cycle_count();
    usage::set_last_cycles(end.wrapping_sub(start));
}

// Ekran güncelleme görevi
#[embassy_executor::task]
async fn display_task() {
    let display = Display::new();
    DISPLAY.call_once(|| display);

    // SOGI-PLL durumunu başlat
    let sogi_state = SOGI_STATE.init(Mutex::new(SogiPllState::new()));
    SOGI_STATE_REF.call_once(|| sogi_state);
    let mut theta_scaled: u16 = 0;
    let mut v_test: i16 = 0;
    loop {
        if let Ok(state) = SOGI_STATE_REF.get().unwrap().lock() {
            theta_scaled = state.theta;
            v_test = state.v_mid;
        }
        if let Some(display) = DISPLAY.get() {
            display.clear();
            let msg = format!(
                "Theta: {:<5}    Cycles: {}",
                theta_scaled,
                usage::get_last_cycles()
            );
            display.write(msg.as_bytes());
        }
        // Hata ayıklama
        extern "C" {
            fn printk(fmt: *const u8, ...);
        }
        unsafe {
            use core::ffi::c_int;
            printk(b"V_TEST: %d \n\0".as_ptr(), v_test as c_int);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

static EXECUTOR_MAIN: StaticCell<Executor> = StaticCell::new();
pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static DISPLAY: spin::Once<Display> = spin::Once::new();
static DAC: spin::Once<Dac> = spin::Once::new();

#[no_mangle]
extern "C" fn rust_main() {
    usage::set_logger_safe().expect("Logger ayarı başarısız");

    let executor = EXECUTOR_MAIN.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(main(spawner)).unwrap();
        spawner.spawn(display_task()).unwrap();
    })
}

#[embassy_executor::task]
async fn main(_spawner: Spawner) {
    let dac = Dac::new();
    DAC.call_once(|| dac);

    // ADC’yi başlat
    let mut adc = Adc::new();
    adc.read_async(
        Duration::from_micros(100).into(), // 10 kHz (100 µs)
        Some(adc_callback),
    );

    if let Some(display) = DISPLAY.get() {
        display.set_backlight(1);
    }

    loop {
        DISPLAY_SIGNAL.wait().await;
    }
}
