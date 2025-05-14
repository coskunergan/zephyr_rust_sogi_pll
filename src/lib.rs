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
const Q15_ONE: i32 = 1 << 15; // 1.0 in Q15 = 32768
const MAX_Q15: u32 = 4095; // DAC max
const MIN_Q15: u32 = 0; // DAC min

// Q15 çarpma: (a * b) >> 15
#[inline]
fn q15_mul(a: i32, b: i32) -> i32 {
    let a_clamped = a.clamp(-Q15_ONE, Q15_ONE);
    let b_clamped = b.clamp(-Q15_ONE, Q15_ONE);
    let result = (a_clamped as i64 * b_clamped as i64) >> 15;
    if result > i32::MAX as i64 {
        i32::MAX
    } else if result < i32::MIN as i64 {
        i32::MIN
    } else {
        result as i32
    }
}

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
    k: i32,               // SOGI kazancı (Q15)
    ts: i16,              // Örnekleme süresi (Q15)
    kp: i32,              // PLL oransal kazanç (Q15)
    ki: i32,              // PLL integral kazanç (Q15)
    last_input: i16,      // Son işlenen giriş (Q15)
    last_raw_adc: i16,    // Son ham ADC değeri
    input_buffer: [i16; 4], // Hareketli ortalama için
    buffer_index: usize,   // Buffer indeksi
}

impl SogiPllState {
    const fn new() -> Self {
        // Sabitler
        let k = (2.0 * (Q15_ONE as f32)) as i32;
        let ts = (0.001 * (Q15_ONE as f32)) as i16; // 1000 µs (1 kHz)
        let kp = (0.2 * (Q15_ONE as f32)) as i32;
        let ki = (20.0 * (Q15_ONE as f32)) as i32;

        SogiPllState {
            v_alpha: 0,
            v_beta: 0,
            v_mid: 0,
            omega: 31416, // 2π * 50 Hz, Q15
            theta: 0,
            integral: 0,
            auto_offset_min: 2048 << 4, // 32768
            auto_offset_max: 2048 << 4, // 32768
            k,
            ts,
            kp,
            ki,
            last_input: 0,
            last_raw_adc: 0,
            input_buffer: [0; 4],
            buffer_index: 0,
        }
    }

    // ADC verisini Q15 formatına dönüştür ve filtrele
    fn scale_adc(&mut self, adc: i16) -> i16 {
        // Ham ADC’yi sakla
        self.last_raw_adc = adc;

        // Otomatik ofset
        let offset_value: u32 = ((adc as u32) << 4).try_into().unwrap();
        if offset_value > self.auto_offset_max {
            self.auto_offset_max = offset_value;
        }
        if offset_value < self.auto_offset_min {
            self.auto_offset_min = offset_value;
        }
        self.auto_offset_min = self.auto_offset_min.saturating_add(1);
        self.auto_offset_max = self.auto_offset_max.saturating_sub(1);
        let mid = (self.auto_offset_min as i32 + self.auto_offset_max as i32) >> 5;
        self.v_mid = mid as i16;

        // Ölçeklendirme: (adc - mid) * (32768 / 2048) ≈ 32
        let diff = (adc as i32 - mid).clamp(-1024, 1024); // Daha sıkı sınır
        let scaled = q15_mul(diff << 5, Q15_ONE); // ×32
        let scaled_i16 = TryInto::<i16>::try_into(scaled).unwrap_or(0);

        // Hareketli ortalama filtresi
        self.input_buffer[self.buffer_index] = scaled_i16;
        self.buffer_index = (self.buffer_index + 1) % 4;
        let avg = self.input_buffer.iter().map(|&x| x as i32).sum::<i32>() >> 2;
        let filtered = TryInto::<i16>::try_into(avg).unwrap_or(0);
        self.last_input = filtered;
        filtered
    }

    // SOGI bloğu
    fn sogi_process(&mut self, input: i16) -> (i16, i16) {
        let error = input - self.v_alpha;
        let delta_alpha = q15_mul(
            q15_mul(error as i32, self.k as i32) - q15_mul(self.v_beta as i32, self.omega),
            (self.ts as i32) << 6,
        );
        self.v_alpha += TryInto::<i16>::try_into(delta_alpha).unwrap_or(0);
        let delta_beta = q15_mul(q15_mul(self.v_alpha as i32, self.omega), (self.ts as i32) << 6);
        self.v_beta += TryInto::<i16>::try_into(delta_beta).unwrap_or(0);
        (self.v_alpha, self.v_beta)
    }

    // Faz hatası hesaplama
    fn phase_error(&self, v_beta: i16, theta: u16) -> (i16, i32) {
        let sin_theta = SIN_LUT[(theta >> 8) as usize];
        let raw_error = if v_beta.abs() > 100 {
            let denominator = (v_beta.abs() >> 8) as i32;
            if denominator != 0 {
                q15_mul(-v_beta as i32, sin_theta as i32) / denominator
            } else {
                0
            }
        } else {
            0
        };
        let error = TryInto::<i16>::try_into(raw_error).unwrap_or(0).clamp(-1000, 1000);
        (error, raw_error)
    }

    // PI denetleyici
    fn pi_controller(&mut self, error: i16) -> i16 {
        self.integral += q15_mul(q15_mul(self.ki as i32, error as i32), self.ts as i32);
        self.integral = self.integral.clamp(-1 << 30, 1 << 30); // ±2^30
        TryInto::<i16>::try_into(q15_mul(self.kp as i32, error as i32) + (self.integral >> 15)).unwrap_or(0)
    }

    // SOGI-PLL güncelleme
    fn update(&mut self, adc_input: i16) -> (i16, i16, i32) {
        // ADC ölçekleme
        let input = self.scale_adc(adc_input);
  
        // SOGI bloğu
        let (v_alpha, v_beta) = self.sogi_process(input);

        // Faz hatası
        let (phase_error, _) = self.phase_error(v_beta, self.theta);

        // PI denetleyici
        let omega_delta = self.pi_controller(phase_error);

        // Omega güncelleme
        const NOMINAL_OMEGA: i32 = 31416; // 2π * 50 Hz
        self.omega = (NOMINAL_OMEGA + omega_delta as i32).clamp(25132, 37700); // ±20% sınır

        // Faz açısı güncelleme
        let theta_inc = (self.omega as u32 * self.ts as u32) / 100;
        self.theta = self.theta.wrapping_add(theta_inc as u16);

        (v_alpha, v_beta, self.omega)
    }
}

// Statik SOGI-PLL durumu (Mutex ile Sync)
static SOGI_STATE: StaticCell<Mutex<SogiPllState>> = StaticCell::new();
static SOGI_STATE_REF: Once<&'static Mutex<SogiPllState>> = Once::new();

// ADC geri çağrı fonksiyonu
fn adc_callback(idx: usize, value: i16) {
    let start = usage::get_cycle_count();



    if idx == 0 {
        if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
            // SOGI-PLL güncelleme
            let (v_alpha, v_beta, omega) = state.update(value);

            // DAC çıkışı
            if let Some(dac) = DAC.get() {
                let sin_value = SIN_LUT[(state.theta >> 8) as usize] as u32;
                dac.write(sin_value);
            }
        }
    }

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
    let mut v_mid: i16 = 0;
    let mut v_alpha: i16 = 0;
    let mut v_beta: i16 = 0;
    let mut omega: i32 = 0;
    let mut input: i16 = 0;
    let mut phase_error: i16 = 0;
    let mut raw_adc: i16 = 0;
    let mut raw_error: i32 = 0;

    loop {
        if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
            theta_scaled = state.theta;
            v_mid = state.v_mid;
            v_alpha = state.v_alpha;
            v_beta = state.v_beta;
            omega = state.omega;
            input = state.last_input;
            raw_adc = state.last_raw_adc;
            let (pe, re) = state.phase_error(v_beta, theta_scaled);
            phase_error = pe;
            raw_error = re;
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
            printk(
                b">V_MID: %d, V_ALPHA: %d, V_BETA: %d, OMEGA: %d, THETA: %d, INPUT: %d, P_ERR: %d, RAW_ADC: %d, RAW_ERR: %d\n\0".as_ptr(),                 
                v_mid as c_int,
                v_alpha as c_int,
                v_beta as c_int,
                omega as c_int,
                theta_scaled as c_int,
                input as c_int,
                phase_error as c_int,
                raw_adc as c_int,
                raw_error as c_int,
            );
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
        Duration::from_micros(1000).into(), // 1 kHz (1000 µs)
        Some(adc_callback),
    );

    if let Some(display) = DISPLAY.get() {
        display.set_backlight(1);
    }

    loop {
        DISPLAY_SIGNAL.wait().await;
    }
}