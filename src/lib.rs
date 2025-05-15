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
const Q15: i32 = 1 << 15; // 2^15 = 32768
const PI_Q15: i32 = 102944; // π ≈ 3.141592653589793 * 2^15
const TAU_Q15: i32 = 205887; // 2π ≈ 6.283185307179586 * 2^15
const MAX_Q15: i32 = 134217728; // 4096.0 * 2^15
const T_Q15: i32 = 8; // 1.0 / 4000.0 * 2^15 = 0.00025 * 2^15
const PI: f32 = 3.14159265358979323;
const TAU: f32 = PI * 2.0;
const MAX: i32 = 4096;
const MIN: i32 = 0;
const T: f32 = 1.0 / 4000.0; // 4 kHz
const TARGET_FREQ: f32 = 50.0;
const N_SAMPLE: usize = (1.0 / T / TARGET_FREQ) as usize; // 4000 / 50 = 80

// Q16.15 aritmetik fonksiyonları
const fn float_to_q16_15(value: f32) -> i32 {
    let scaled = value * 32768.0;
    let rounded = if scaled >= 0.0 {
        (scaled + 0.5) as i32
    } else {
        (scaled - 0.5) as i32
    };
    rounded
}

fn q16_15_to_float(value: i32) -> f32 {
    value as f32 / Q15 as f32
}

fn q16_15_mul(a: i32, b: i32) -> i32 {
    let temp: i64 = (a as i64) * (b as i64);
    let result = (temp >> 15) as i32;
    if temp > i64::MAX >> 15 || temp < i64::MIN >> 15 {
        if result >= 0 {
            i32::MAX
        } else {
            i32::MIN
        }
    } else {
        result
    }
}

fn q16_15_add(a: i32, b: i32) -> i32 {
    a.saturating_add(b)
}

fn q16_15_sub(a: i32, b: i32) -> i32 {
    a.saturating_sub(b)
}

fn q16_15_div(a: i32, b: i32) -> i32 {
    if b == 0 {
        return 0; // Bölme sıfıra karşı koruma
    }
    let temp: i64 = (a as i64) << 15;
    (temp / b as i64) as i32
}

// SOGI-PLL durum yapısı
struct SogiPllState {
    pid: PidState,
    launch_loop: bool,
    sample_index: u16,
    omega: f32,
    cur_phase: i32,
    auto_offset_min: i32,
    auto_offset_max: i32,
    sogi_s1: i32,
    sogi_s2: i32,
    last_error: f32,
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
                kp: 1638400,      // 50.0 * 2^15
                ki: 65536,        // 2.0 * 2^15
                kc: 32768,        // 1.0 * 2^15
                i_min: -13508845, // -(50.0 + 15.0) * 2π * 2^15
                i_max: 13508845,  // (50.0 + 15.0) * 2π * 2^15
            },
            launch_loop: false,
            sample_index: 0,
            omega: TARGET_FREQ * TAU,
            cur_phase: 0,
            auto_offset_min: MAX,
            auto_offset_max: MIN,
            sogi_s1: 0,
            sogi_s2: 0,
            last_error: 0.0,
        }
    }

    fn reset(&mut self) {
        self.pid.i_sum = 0;
        self.pid.sat_err = 0;
        self.launch_loop = false;
        self.sample_index = 0;
        self.omega = TARGET_FREQ * TAU;
        self.cur_phase = 0;
        self.auto_offset_min = MAX;
        self.auto_offset_max = MIN;
        self.sogi_s1 = 0;
        self.sogi_s2 = 0;
        self.last_error = 0.0;
    }

    fn is_lock(&self, th: i32) -> bool {
        /*self.launch_loop && float_to_q16_15(self.last_error.abs()) < th*/
        true
    }
}

fn fast_sin(theta: f32) -> f32 {
    q16_15_to_float(fast_sinn(float_to_q16_15(theta)))
}

// Hızlı sinüs yaklaşıklığı (Q16.15 formatında)
fn fast_sinn(theta: i32) -> i32 {
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

    let x_scaled = q16_15_mul(x, 25735); // 51471.0 / 65536.0 * 2^15 ≈ 0.7854 * 2^15
    let x2 = q16_15_mul(x_scaled, x_scaled);
    let x3 = q16_15_mul(x2, x_scaled);
    let sin_x = q16_15_sub(x_scaled, q16_15_mul(x3, 5456)); // 5461.0 / 32768.0 * 2^15 ≈ 0.1666 * 2^15

    q16_15_mul(sin_x, sign * Q15)
}

// Hızlı kosinüs (sin’den türetilmiş)
fn fast_cos(theta: f32) -> f32 {
    q16_15_to_float(fast_coss(float_to_q16_15(theta)))
}

// Hızlı kosinüs
fn fast_coss(theta: i32) -> i32 {
    fast_sinn(q16_15_add(theta, PI_Q15 / 2))
}

fn pi_transfer(e: f32, pid: &mut PidState) -> f32 {
    let sat = q16_15_add(q16_15_mul(pid.kp, float_to_q16_15(e)), pid.i_sum);
    let out = if sat > pid.i_max {
        pid.i_max
    } else if sat < pid.i_min {
        pid.i_min
    } else {
        sat
    };

    pid.sat_err = q16_15_sub(out, sat);
    pid.i_sum = q16_15_add(
        pid.i_sum,
        q16_15_add(
            q16_15_mul(pid.ki, float_to_q16_15(e)),
            q16_15_mul(pid.kc, pid.sat_err),
        ),
    );

    if pid.i_sum > pid.i_max {
        pid.i_sum = pid.i_max;
    } else if pid.i_sum < pid.i_min {
        pid.i_sum = pid.i_min;
    }

    q16_15_to_float(out)
}

// SOGI-PLL transfer
fn spll_transfer_1phase(val: i32, state: &mut SogiPllState) {
    // Otomatik ofset
    let v_org = {
        state.auto_offset_max = q16_15_sub(state.auto_offset_max, 13421); // 4096.0 / 1e4 * 2^15
        state.auto_offset_min = q16_15_add(state.auto_offset_min, 13421); // 4096.0 / 1e4 * 2^15
        if val * 32768 > state.auto_offset_max {
            state.auto_offset_max = val * 32768;
        }
        if val * 32768 < state.auto_offset_min {
            state.auto_offset_min = val * 32768;
        }
        let mid = q16_15_mul(
            q16_15_add(state.auto_offset_min, state.auto_offset_max),
            16384,
        ); // 0.5 * 2^15
        q16_15_sub(val * 32768, mid)
    };

    // Örnekleme kontrolü
    state.sample_index = state.sample_index.wrapping_add(1); // Sürekli artır

    if state.sample_index < N_SAMPLE as u16 {
        state.launch_loop = false;
        return;
    } else {
        state.launch_loop = true;
    }    

    // Normalizasyon
    let denom = q16_15_sub(state.auto_offset_max, state.auto_offset_min).max(3); // ~ 1e-5
    let v = q16_15_div(v_org, denom);          

    ////////////////////////////////////////////////////////////
    // SOGI
    const K_Q15: i32 = 46334; // 1.414 * 2^15
    const W_Q15: i32 = 10294367; // 50.0 * 2π * 2^15
    const W: f32 = TARGET_FREQ * TAU;
    let sogi_u = q16_15_mul(
        q16_15_mul(
            K_Q15,
            q16_15_sub(q16_15_sub(v, state.sogi_s1), state.sogi_s2),
        ),
        W_Q15,
    );
    state.sogi_s1 = q16_15_add(state.sogi_s1, q16_15_mul(T_Q15, sogi_u));
    state.sogi_s2 = q16_15_add(
        state.sogi_s2,
        q16_15_mul(T_Q15, q16_15_mul(W_Q15, state.sogi_s1)),
    );
    ////////////////////////////////////////////////////////////
    // VCO (Park)
    let ua = q16_15_to_float(state.sogi_s1);
    let ub = q16_15_to_float(state.sogi_s2);
    //let theta = (360.0 / TAU) * state.cur_phase;
    let theta = q16_15_to_float(q16_15_mul(state.cur_phase, 1878420)); // 360.0 / (2π) * 2^15 ≈ 57.2958 * 2^15
                                                                                        //let st = fast_sin(theta * TAU / 360.0);
    let st = q16_15_to_float(fast_sinn(q16_15_mul(float_to_q16_15(theta), 571))); // (2π / 360.0) * 2^15 ≈ 0.0174533 * 2^15
                                                                                  //let ct = fast_cos(theta * TAU / 360.0);
    let ct = q16_15_to_float(fast_coss(q16_15_mul(float_to_q16_15(theta), 571))); // (2π / 360.0) * 2^15 ≈ 0.0174533 * 2^15
    let uq = ct * ub - st * ua;

    // PI
    let e = 0.0 - uq;
    let u = pi_transfer(e, &mut state.pid);

    // Faz güncelleme
    let mut i = q16_15_to_float(state.cur_phase) + T * u;
    if i > TAU {
        i -= TAU;
    } else if i < -TAU {
        i += TAU;
    }

    state.omega = u;
    state.cur_phase = float_to_q16_15(i);
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
            spll_transfer_1phase(value as i32, &mut state);

            // DAC çıkışı (0-4095 aralığında)
            if let Some(dac) = DAC.get() {
                let sin_value = fast_sinn(q16_15_add(state.cur_phase, 2949120)); // 90.0 * 57.2958 * 2^15
                let amplitude = if state.is_lock(33) {
                    // 1e-3 * 2^15 ≈ 0.032768 * 2^15
                    let amp =
                        q16_15_to_float(q16_15_sub(state.auto_offset_max, state.auto_offset_min))
                            * 0.5;
                    (amp * 2048.0).min(2048.0) // Maksimum genlik 2048
                } else {
                    2048.0
                };
                let result = (q16_15_to_float(sin_value) * amplitude + 2048.0).clamp(0.0, 4095.0);
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
            theta_scaled = (q16_15_to_float(q16_15_mul(state.cur_phase, 318)) as u16).min(65535);
            // 65535.0 / (2π) * 2^15
        }
        if let Some(display) = DISPLAY.get() {
            display.clear();
            let msg = format!("T:{:<5} C:{:<6}", theta_scaled, usage::get_last_cycles());
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
