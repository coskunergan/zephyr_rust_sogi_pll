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

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
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

// Sabitler
const PI: f32 = 3.14159265358979323;
const TAU: f32 = PI * 2.0;
const MAX: f32 = 4096.0;
const MIN: f32 = 0.0;
const T: f32 = 1.0 / 1000.0; // 1 kHz
const TARGET_FREQ: f32 = 50.0;
const N_SAMPLE: usize = (1.0 / T / TARGET_FREQ) as usize; // 1000 / 50 = 20

// SOGI-PLL durum yapısı (C++ SPLL’den uyarlandı)
struct SogiPllState {
    pid: PidState,
    launch_loop: bool,
    sample_index: u16,
    omega: f32,
    cur_phase: f32,
    auto_offset_min: f32,
    auto_offset_max: f32,
    sogi_s1: f32,
    sogi_s2: f32,
    last_error: f32,
}

struct PidState {
    i_sum: f32,
    sat_err: f32,
    kp: f32,
    ki: f32,
    kc: f32,
    i_min: f32,
    i_max: f32,
}

impl SogiPllState {
    const fn new() -> Self {
        SogiPllState {
            pid: PidState {
                i_sum: 0.0,
                sat_err: 0.0,
                kp: 750.0,
                ki: 15.0,
                kc: 10.0,
                i_min: -(TARGET_FREQ + 15.0) * TAU,
                i_max: (TARGET_FREQ + 15.0) * TAU,
            },
            launch_loop: false,
            sample_index: 0,
            omega: TARGET_FREQ * TAU, // 50 * 2π
            cur_phase: 0.0,
            auto_offset_min: MAX,
            auto_offset_max: MIN,
            sogi_s1: 0.0,
            sogi_s2: 0.0,
            last_error: 0.0,
        }
    }

    fn reset(&mut self) {
        self.pid.i_sum = 0.0;
        self.pid.sat_err = 0.0;
        self.launch_loop = false;
        self.sample_index = 0;
        self.omega = TARGET_FREQ * TAU;
        self.cur_phase = 0.0;
        self.auto_offset_min = MAX;
        self.auto_offset_max = MIN;
        self.sogi_s1 = 0.0;
        self.sogi_s2 = 0.0;
        self.last_error = 0.0;
    }

    fn is_lock(&self, th: f32) -> bool {
        /*self.launch_loop &&*/ self.last_error.abs() < th
    }
}

// Hızlı sinüs yaklaşıklığı (Q15, 0..2π için, Rust’tan uyarlandı)
fn fast_sin(theta: f32) -> f32 {
    let theta = theta % TAU; // 0..2π
    let mut sign = 1.0;
    let mut x = theta;

    if theta >= PI / 2.0 && theta < PI {
        x = PI - theta;
    } else if theta >= PI && theta < PI + PI / 2.0 {
        x = theta - PI;
        sign = -1.0;
    } else if theta >= PI + PI / 2.0 {
        x = TAU - theta;
        sign = -1.0;
    }

    let x = x * 51471.0 / 65536.0;
    let x2 = x * x;
    let x3 = x2 * x;
    let sin_x = x - (x3 * 5461.0 / 32768.0);

    sin_x * sign
}

// Hızlı kosinüs (sin’den türetilmiş)
fn fast_cos(theta: f32) -> f32 {
    fast_sin(theta + PI / 2.0)
}

// PID PI transfer (C++ PID::pi_transfer’den)
fn pi_transfer(e: f32, pid: &mut PidState) -> f32 {
    let sat = pid.kp * e + pid.i_sum;
    let out = if sat > pid.i_max {
        pid.i_max
    } else if sat < pid.i_min {
        pid.i_min
    } else {
        sat
    };

    pid.sat_err = out - sat;
    pid.i_sum += pid.ki * e + pid.kc * pid.sat_err;

    if pid.i_sum > pid.i_max {
        pid.i_sum = pid.i_max;
    } else if pid.i_sum < pid.i_min {
        pid.i_sum = pid.i_min;
    }

    out
}

// SOGI-PLL transfer (C++ SPLL::transfer_1phase’ten)
fn spll_transfer_1phase(val: f32, state: &mut SogiPllState) {
    // Otomatik ofset
    let v_org = {
        state.auto_offset_max -= MAX / 1e4;
        state.auto_offset_min += MAX / 1e4;
        if val > state.auto_offset_max {
            state.auto_offset_max = val;
        } else if val < state.auto_offset_min {
            state.auto_offset_min = val;
        }
        let mid = (state.auto_offset_min + state.auto_offset_max) * 0.5;
        val - mid
    };

    // Örnekleme kontrolü
    if state.sample_index < N_SAMPLE as u16 {
        state.sample_index += 1;
        state.launch_loop = false;
        return;
    } else {
        state.launch_loop = true;
    }

    // Normalizasyon
    let v = v_org / (state.auto_offset_max - state.auto_offset_min).max(1e-6);

    // SOGI
    const K: f32 = 1.414;
    const W: f32 = TARGET_FREQ * TAU;
    let sogi_u = (K * (v - state.sogi_s1) - state.sogi_s2) * W;
    state.sogi_s1 += T * sogi_u;
    state.sogi_s2 += T * W * state.sogi_s1;

    // VCO (Park)
    let ua = state.sogi_s1;
    let ub = state.sogi_s2;
    let theta = (360.0 / TAU) * state.cur_phase;
    let st = fast_sin(theta * TAU / 360.0);
    let ct = fast_cos(theta * TAU / 360.0);
    let uq = ct * ub - st * ua;

    // PI
    let e = 0.0 - uq;
    let u = pi_transfer(e, &mut state.pid);

    // Faz güncelleme
    let mut i = state.cur_phase + T * u;
    if i > TAU {
        i -= TAU;
    } else if i < -TAU {
        i += TAU;
    }

    state.omega = u;
    state.cur_phase = i;
    state.last_error = e;
}

// Statik SOGI-PLL durumu
static SOGI_STATE: StaticCell<Mutex<SogiPllState>> = StaticCell::new();
static SOGI_STATE_REF: Once<&'static Mutex<SogiPllState>> = Once::new();

// ADC geri çağrı fonksiyonu
fn adc_callback(idx: usize, value: i16) {
    // Performans ölçümü
    let start = usage::get_cycle_count();

    if idx == 0 {
        if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
            // ADC girişini ölçekle (12-bit: 0..4095 → 0..4096)
            let scaled_value = value as f32 * MAX / 4095.0;

            // SOGI-PLL
            spll_transfer_1phase(scaled_value, &mut state);

            // DAC çıkışı
            if let Some(dac) = DAC.get() {
                let sin_value = fast_sin(state.cur_phase);
                let amplitude = if state.is_lock(50e-3) {
                    // Genlik: (auto_offset_max - auto_offset_min) ölçeklenmiş
                    (state.auto_offset_max - state.auto_offset_min).max(1.0) * 0.5
                } else {
                    2048.0 // Sabit genlik fallback
                };
                let dac_value = (sin_value * amplitude + 2048.0).clamp(0.0, 4095.0) as u32;
                dac.write(dac_value);
            }
        }
    }

    // Performans ölçümü
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

    // if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
    //     state.reset();
    // }
    loop {
        if let Ok(state) = SOGI_STATE_REF.get().unwrap().lock() {
            if let Some(display) = DISPLAY.get() {
                display.clear();
                let theta_scaled = (state.cur_phase / TAU * 65535.0) as u16;
                let msg = format!(
                    "Theta: {:<5}    Cycles: {} L:{}",
                    theta_scaled,
                    usage::get_last_cycles(),
                    0
                );
                display.write(msg.as_bytes());
            }
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

    // if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
    //     state.reset();
    // }

    // ADC’yi başlat
    let mut adc = Adc::new();
    adc.read_async(
        Duration::from_micros(250).into(), // 4 kHz
        Some(adc_callback),
    );

    if let Some(display) = DISPLAY.get() {
        display.set_backlight(1);
    }

    loop {
        DISPLAY_SIGNAL.wait().await;
    }
}
