// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#![no_std]

extern crate alloc;

use alloc::format;
use core::convert::TryInto;
use core::ffi::c_double;
use embassy_time::{Duration, Timer};

#[cfg(feature = "executor-thread")]
use embassy_executor::Executor;

#[cfg(feature = "executor-zephyr")]
use zephyr::embassy::Executor;

use core::f32::consts;
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
const Q15_SHIFT: i32 = 15;
const Q15_SCALE: f32 = (1 << Q15_SHIFT) as f32; // Q15 ölçeklendirme faktörü (2^15)
const TWO_PI: f32 = consts::PI * 2.0; // 2π ≈ 6.283185307179586

// Frekans değişkenleri
const SAMPLE_FREQ: f32 = 4000.0; // 4 KHz
const TARGET_FREQ: f32 = 50.0; // 50 Hz

// Q15 formatında temel sabitler
const PI_Q15: i32 = (consts::PI * Q15_SCALE) as i32; // π * 2^15 ≈ 102944
const TAU_Q15: i32 = (TWO_PI * Q15_SCALE) as i32; // 2π * 2^15 ≈ 205887
const T_Q15: i32 = ((1.0 / SAMPLE_FREQ as f32) * Q15_SCALE) as i32;

// Giriş Limit Değerleri
const INPUT_MAX: i32 = 4095;
const INPUT_MIN: i32 = 0;

const T: f32 = 1.0 / SAMPLE_FREQ as f32; // x kHz
const N_SAMPLE: usize = (1.0 / T / TARGET_FREQ) as usize;

// PID parametreleri (algoritmik olarak belirlenmiş)
const PID_KP_FLOAT: f32 = 750.0; // Proportional kazanç
const PID_KI_FLOAT: f32 = 2.0; // Integral kazanç
const PID_KC_FLOAT: f32 = 1.0; // Anti-windup kazancı
const PID_FREQ_RANGE: f32 = 25.0; // Frekans sapma aralığı (±25 Hz)
const NOMINAL_FREQ: f32 = 50.0; // Nominal frekans (50 Hz)

// Otomatik ofset sınırları (algoritmik olarak belirlenmiş)
const OFFSET_STEP_COEFF: i32 = 10000; // Ofset güncelleme adımı

// SOGI parametreleri
const SOGI_K_FLOAT: f32 = 1.4142135623730951; // SOGI kazancı (√2)
const ANGLE_TO_RAD_SCALE: f32 = 360.0 / TWO_PI; // Dereceyi radyana çevirme (360/(2π) ≈ 57.2958)
const RAD_TO_ANGLE_SCALE: f32 = TWO_PI / 360.0; // Radyanı dereceye çevirme (2π/360 ≈ 0.0174533)

// PID_I_MIN ve PID_I_MAX için hassas hesaplama
const FREQ_LIMIT: f32 = NOMINAL_FREQ + PID_FREQ_RANGE;
const ANGULAR_FREQ: f32 = FREQ_LIMIT * TWO_PI;
const PID_I_MAX: i32 = (ANGULAR_FREQ * Q15_SCALE) as i32;
const PID_I_MIN: i32 = (-ANGULAR_FREQ * Q15_SCALE) as i32;

// Q15 formatında diğer sabitler
const PID_KP: i32 = (PID_KP_FLOAT * Q15_SCALE) as i32; // 750.0 * 2^15 ≈ 24576000
const PID_KI: i32 = (PID_KI_FLOAT * Q15_SCALE) as i32; // 2.0 * 2^15 ≈ 65536
const PID_KC: i32 = (PID_KC_FLOAT * Q15_SCALE) as i32; // 1.0 * 2^15 ≈ 32768
const INITIAL_OMEGA: i32 = (NOMINAL_FREQ * TWO_PI * Q15_SCALE) as i32; // 50 * 2π * 2^15 ≈ 10294367
const OFFSET_STEP: i32 = (((INPUT_MAX - INPUT_MIN) * Q15_SCALE as i32) / OFFSET_STEP_COEFF) as i32;
const SOGI_K: i32 = (SOGI_K_FLOAT * Q15_SCALE) as i32; // √2 * 2^15 ≈ 46334
const ANGLE_TO_RAD: i32 = (ANGLE_TO_RAD_SCALE * Q15_SCALE) as i32; // 360/(2π) * 2^15 ≈ 1878420
const RAD_TO_ANGLE: i32 = (RAD_TO_ANGLE_SCALE * Q15_SCALE) as i32; // 2π/360 * 2^15 ≈ 571
const HALF_SCALE: i32 = (0.5 * Q15_SCALE) as i32; // 0.5 * 2^15 ≈ 16384
const DEFAULT_DENOM: i32 = (1.0 * Q15_SCALE) as i32; // 1.0 * 2^15 ≈ 32768

// Faz ofseti (90 derece)
const PHASE_OFFSET_90: i32 = (90.0 * Q15_SCALE) as i32; // 90 * 2^15 ≈ 2949120

// Ekran için faz skalası
const THETA_SCALE: i32 = (360.0 / (TWO_PI * 180.0) * Q15_SCALE) as i32; // Faz açısını ekrana uygun ölçeklendirme ≈ 318

// Q16.15 aritmetik fonksiyonları
fn q15_to_float(value: i32) -> f32 {
    value as f32 / Q15_SCALE
}

fn q15_mul(a: i32, b: i32) -> i32 {
    (((a as i64) * (b as i64)) >> Q15_SHIFT) as i32
}

fn q15_add(a: i32, b: i32) -> i32 {
    a.saturating_add(b)
}

fn q15_sub(a: i32, b: i32) -> i32 {
    a.saturating_sub(b)
}

fn q15_div(a: i32, b: i32) -> i32 {
    if b == 0 {
        return 0;
    }
    (((a as i64) << Q15_SHIFT) / b as i64) as i32
}

// SOGI-PLL durum yapısı
struct SogiPllState {
    pid: PidState,
    launch_loop: bool,
    sample_index: u16,
    omega: i32,
    cur_phase: i32,
    auto_offset_min: i32,
    auto_offset_max: i32,
    sogi_s1: i32,
    sogi_s2: i32,
    last_error: i32,
}

struct PidState {
    i_sum: i32,
    sat_err: i32,
    kp: i32,
    ki: i32,
    kc: i32,
    i_min: i32,
    i_max: i32,
}

impl SogiPllState {
    const fn new() -> Self {
        SogiPllState {
            pid: PidState {
                i_sum: 0,
                sat_err: 0,
                kp: PID_KP,
                ki: PID_KI,
                kc: PID_KC,
                i_min: PID_I_MIN,
                i_max: PID_I_MAX,
            },
            launch_loop: false,
            sample_index: 0,
            omega: INITIAL_OMEGA,
            cur_phase: 0,
            auto_offset_min: INPUT_MAX * Q15_SCALE as i32,
            auto_offset_max: INPUT_MIN * Q15_SCALE as i32,
            sogi_s1: 0,
            sogi_s2: 0,
            last_error: 0,
        }
    }

    fn reset(&mut self) {
        self.pid.i_sum = 0;
        self.pid.sat_err = 0;
        self.launch_loop = false;
        self.sample_index = 0;
        self.omega = INITIAL_OMEGA;
        self.cur_phase = 0;
        self.auto_offset_min = INPUT_MAX * Q15_SCALE as i32;
        self.auto_offset_max = INPUT_MIN * Q15_SCALE as i32;
        self.sogi_s1 = 0;
        self.sogi_s2 = 0;
        self.last_error = 0;
    }

    fn is_lock(&self, th: i32) -> bool {
        self.launch_loop && self.last_error.abs() < th
    }
}

// Hızlı sinüs yaklaşıklığı (Q16.15 formatında)
fn fast_sin(theta: i32) -> i32 {
    const SIN_COEFF: i32 = ((consts::PI / 4.0) * Q15_SCALE) as i32; // π/4 * 2^15 ≈ 25736
    const SIN_CUBIC_COEFF: i32 = ((1.0 / 6.0) * Q15_SCALE) as i32; // 1/6 * 2^15 ≈ 5461

    let mut theta = theta % TAU_Q15; // 0..2π
    if theta < 0 {
        theta += TAU_Q15;
    }
    let mut sign = 1;
    let mut x = theta;

    if theta >= PI_Q15 / 2 && theta < PI_Q15 {
        x = PI_Q15 - theta;
    } else if theta >= PI_Q15 && theta < PI_Q15 + PI_Q15 / 2 {
        x = theta - PI_Q15;
        sign = -1;
    } else if theta >= PI_Q15 + PI_Q15 / 2 {
        x = TAU_Q15 - theta;
        sign = -1;
    }

    let x_scaled = q15_mul(x, SIN_COEFF);
    let x2 = q15_mul(x_scaled, x_scaled);
    let x3 = q15_mul(x2, x_scaled);
    let sin_x = q15_sub(x_scaled, q15_mul(x3, SIN_CUBIC_COEFF));

    q15_mul(sin_x, sign * Q15_SCALE as i32)
}

// Hızlı kosinüs
fn fast_cos(theta: i32) -> i32 {
    fast_sin(q15_add(theta, PI_Q15 / 2))
}

fn pi_transfer(e: i32, pid: &mut PidState) -> i32 {
    let sat = q15_add(q15_mul(pid.kp, e), pid.i_sum);
    let out = if sat > pid.i_max {
        pid.i_max
    } else if sat < pid.i_min {
        pid.i_min
    } else {
        sat
    };
    pid.sat_err = q15_sub(out, sat);
    pid.i_sum = q15_add(
        pid.i_sum,
        q15_add(q15_mul(pid.ki, e), q15_mul(pid.kc, pid.sat_err)),
    );
    if pid.i_sum > pid.i_max {
        pid.i_sum = pid.i_max;
    } else if pid.i_sum < pid.i_min {
        pid.i_sum = pid.i_min;
    }
    out
}

// SOGI-PLL transfer
fn spll_transfer_1phase(val: i32, state: &mut SogiPllState) {
    // Otomatik ofset
    let v_org = {
        state.auto_offset_max = q15_sub(state.auto_offset_max, OFFSET_STEP);
        state.auto_offset_min = q15_add(state.auto_offset_min, OFFSET_STEP);
        if val > state.auto_offset_max {
            state.auto_offset_max = val;
        }
        if val < state.auto_offset_min {
            state.auto_offset_min = val;
        }
        let mid = q15_mul(q15_add(state.auto_offset_min, state.auto_offset_max), HALF_SCALE); // 0.5 * 2^15
        q15_sub(val, mid)
    };

    // Örnekleme kontrolü
    state.sample_index = state.sample_index.wrapping_add(1);

    if state.sample_index < N_SAMPLE as u16 {
        state.launch_loop = false;
        return;
    } else {
        state.launch_loop = true;
    }

    // Normalizasyon
    let denom = q15_sub(state.auto_offset_max, state.auto_offset_min).max(DEFAULT_DENOM);
    let v = q15_div(v_org, denom);

    // SOGI
    const K_Q15: i32 = 46334; // 1.414 * 2^15
    const W_Q15: i32 = 10294367; // 50.0 * 2π * 2^15
    let sogi_u = q15_mul(
        q15_sub(q15_mul(K_Q15, q15_sub(v, state.sogi_s1)), state.sogi_s2),
        W_Q15,
    );
    state.sogi_s1 = q15_add(state.sogi_s1, q15_mul(T_Q15, sogi_u));
    state.sogi_s2 = q15_add(state.sogi_s2, q15_mul(T_Q15, q15_mul(W_Q15, state.sogi_s1)));

    // VCO (Park)
    let ua = state.sogi_s1;
    let ub = state.sogi_s2;
    let theta = q15_mul(state.cur_phase, ANGLE_TO_RAD);
    let st = fast_sin(q15_mul(theta, RAD_TO_ANGLE));
    let ct = fast_cos(q15_mul(theta, RAD_TO_ANGLE));
    let uq = q15_sub(q15_mul(ct, ub), q15_mul(st, ua));

    // PI
    let e = q15_sub(0, uq);
    let u = pi_transfer(e, &mut state.pid);

    // Faz güncelleme
    let mut i = q15_add(state.cur_phase, q15_mul(T_Q15, u));
    if i > TAU_Q15 {
        i = q15_sub(i, TAU_Q15);
    } else if i < -TAU_Q15 {
        i = q15_add(i, TAU_Q15);
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
    let start = usage::get_cycle_count();

    if idx == 0 {
        if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
            // SOGI-PLL
            spll_transfer_1phase(value as i32 * Q15_SCALE as i32, &mut state);

            // DAC çıkışı (0-4095 aralığında)
            if let Some(dac) = DAC.get() {
                let sin_value = fast_sin(q15_add(state.cur_phase, PHASE_OFFSET_90)); 
                let amplitude = if state.is_lock(33) {
                    let amp =
                        q15_to_float(q15_sub(state.auto_offset_max, state.auto_offset_min)) * 0.5;
                    (amp * 2048.0).min(2048.0) // Maksimum genlik 2048
                } else {
                    2048.0
                };
                let result = (q15_to_float(sin_value) * amplitude + 2048.0).clamp(0.0, 4095.0);
                let dac_value = result as u32;
                dac.write(dac_value);
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

    let sogi_state = SOGI_STATE.init(Mutex::new(SogiPllState::new()));
    SOGI_STATE_REF.call_once(|| sogi_state);
    let mut theta_scaled: u16 = 0;
    loop {
        if let Ok(state) = SOGI_STATE_REF.get().unwrap().lock() {
            theta_scaled = (q15_to_float(q15_mul(state.cur_phase, THETA_SCALE)) as u16).min(65535);
        }
        if let Some(display) = DISPLAY.get() {
            display.clear();
            let msg = format!(
                "T:{:<5} C:{:<6} M: {}",
                theta_scaled,
                usage::get_last_cycles(),
                INITIAL_OMEGA
            );
            display.write(msg.as_bytes());
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

    let mut adc = Adc::new();
    adc.read_async_isr(
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
