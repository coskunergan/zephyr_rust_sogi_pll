// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#![no_std]

extern crate alloc;

use alloc::format;
use embassy_time::{Duration, Timer};

#[cfg(feature = "executor-thread")]
use embassy_executor::Executor;

#[cfg(feature = "executor-zephyr")]
use zephyr::embassy::Executor;

use core::cell::OnceCell;
use core::f32::consts;
use critical_section::Mutex as CriticalMutex;
use embassy_executor::Spawner;
use static_cell::StaticCell;
use zephyr::sync::Mutex;

use adc_io::Adc;
use dac_io::Dac;
use display_io::Display;

mod adc_io;
mod dac_io;
mod display_io;
mod usage;

const Q15_SHIFT: i32 = 15;
const Q15_SCALE: f32 = (1 << Q15_SHIFT) as f32;
const TWO_PI: f32 = consts::PI * 2.0;
const SAMPLE_FREQ: f32 = 2500.0;
const TARGET_FREQ: f32 = 50.0;
const PI_Q15: i32 = (consts::PI * Q15_SCALE) as i32;
const TAU_Q15: i32 = (TWO_PI * Q15_SCALE) as i32;
const T_Q15: i32 = ((1.0 / SAMPLE_FREQ as f32) * Q15_SCALE) as i32;
const INPUT_MAX: i32 = 4095;
const INPUT_MIN: i32 = 0;
const T: f32 = 1.0 / SAMPLE_FREQ as f32;
const N_SAMPLE: usize = (1.0 / T / TARGET_FREQ) as usize;
const PID_KP_FLOAT: f32 = 150.0;
const PID_KI_FLOAT: f32 = 15.0;
const PID_KC_FLOAT: f32 = 10.0;
const PID_FREQ_RANGE: f32 = 15.0;
const NOMINAL_FREQ: f32 = 50.0;
const OFFSET_STEP_COEFF: i32 = (1e4) as i32;

const SOGI_K_FLOAT: f32 = 1.4142135623730951;
const ANGLE_TO_RAD_SCALE: f32 = 360.0 / TWO_PI;
const RAD_TO_ANGLE_SCALE: f32 = TWO_PI / 360.0;
const FREQ_LIMIT: f32 = NOMINAL_FREQ + PID_FREQ_RANGE;
const ANGULAR_FREQ: f32 = FREQ_LIMIT * TWO_PI;
const PID_I_MAX: i32 = (ANGULAR_FREQ * Q15_SCALE) as i32;
const PID_I_MIN: i32 = (-ANGULAR_FREQ * Q15_SCALE) as i32;
const PID_KP: i32 = (PID_KP_FLOAT * Q15_SCALE) as i32;
const PID_KI: i32 = (PID_KI_FLOAT * Q15_SCALE) as i32;
const PID_KC: i32 = (PID_KC_FLOAT * Q15_SCALE) as i32;
const INITIAL_OMEGA: i32 = (NOMINAL_FREQ * TWO_PI * Q15_SCALE) as i32;
const OFFSET_STEP: i32 = (((INPUT_MAX - INPUT_MIN) * Q15_SCALE as i32) / OFFSET_STEP_COEFF) as i32;
const SOGI_K: i32 = (SOGI_K_FLOAT * Q15_SCALE) as i32;
const ANGLE_TO_RAD: i32 = (ANGLE_TO_RAD_SCALE * Q15_SCALE) as i32;
const RAD_TO_ANGLE: i32 = (RAD_TO_ANGLE_SCALE * Q15_SCALE) as i32;
const HALF_SCALE: i32 = (0.5 * Q15_SCALE) as i32;
const DEFAULT_DENOM: i32 = (1.0 * Q15_SCALE) as i32;
const THETA_SCALE: i32 = (360.0 / (TWO_PI * 180.0) * Q15_SCALE) as i32;
const PHASE_OFFSET: i32 = ((consts::PI / 2.0) * Q15_SCALE) as i32;

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
    duration_ns: u32,
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
            duration_ns: 0,
        }
    }
    #[allow(dead_code)]
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

fn fast_sin(theta: i32) -> i32 {
    const SIN_COEFF: i32 = ((consts::PI / 4.0) * Q15_SCALE) as i32;
    const SIN_CUBIC_COEFF: i32 = ((1.0 / 6.0) * Q15_SCALE) as i32;

    let mut theta = theta % TAU_Q15;
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

fn spll_update(val: i32, state: &mut SogiPllState) {
    let v_org = {
        state.auto_offset_max = q15_sub(state.auto_offset_max, OFFSET_STEP);
        state.auto_offset_min = q15_add(state.auto_offset_min, OFFSET_STEP);
        if val > state.auto_offset_max {
            state.auto_offset_max = val;
        }
        if val < state.auto_offset_min {
            state.auto_offset_min = val;
        }
        let mid = q15_mul(
            q15_add(state.auto_offset_min, state.auto_offset_max),
            HALF_SCALE,
        );
        q15_sub(val, mid)
    };

    state.sample_index = state.sample_index.wrapping_add(1);

    if state.sample_index < N_SAMPLE as u16 {
        state.launch_loop = false;
        return;
    } else {
        state.launch_loop = true;
    }

    let denom = q15_sub(state.auto_offset_max, state.auto_offset_min).max(DEFAULT_DENOM);
    let v = q15_div(v_org, denom);

    let sogi_u = q15_mul(
        q15_sub(q15_mul(SOGI_K, q15_sub(v, state.sogi_s1)), state.sogi_s2),
        INITIAL_OMEGA,
    );
    state.sogi_s1 = q15_add(state.sogi_s1, q15_mul(T_Q15, sogi_u));
    state.sogi_s2 = q15_add(
        state.sogi_s2,
        q15_mul(T_Q15, q15_mul(INITIAL_OMEGA, state.sogi_s1)),
    );

    let ua = state.sogi_s1;
    let ub = state.sogi_s2;
    let theta = q15_mul(state.cur_phase, ANGLE_TO_RAD);
    let st = fast_sin(q15_mul(theta, RAD_TO_ANGLE));
    let ct = fast_cos(q15_mul(theta, RAD_TO_ANGLE));
    let uq = q15_sub(q15_mul(ct, ub), q15_mul(st, ua));

    let e = q15_sub(0, uq);
    let u = pi_transfer(e, &mut state.pid);

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

static SOGI_STATE: StaticCell<Mutex<SogiPllState>> = StaticCell::new();
static SOGI_STATE_REF: CriticalMutex<OnceCell<&'static Mutex<SogiPllState>>> =
    CriticalMutex::new(OnceCell::new());
static DAC_STATE: StaticCell<Mutex<Dac>> = StaticCell::new();
static DAC_STATE_REF: CriticalMutex<OnceCell<&'static Mutex<Dac>>> =
    CriticalMutex::new(OnceCell::new());

fn adc_callback(idx: usize, value: i16) {
    if idx == 0 {
        critical_section::with(|cs| {
            if let Some(state_ref) = SOGI_STATE_REF.borrow(cs).get() {
                let mut state = state_ref.lock().unwrap();
                state.duration_ns = usage::measure_function_duration_ns(|| {
                    spll_update(value as i32 * Q15_SCALE as i32, &mut state)
                }) as u32;

                if let Some(dac_ref) = DAC_STATE_REF.borrow(cs).get() {
                    let dac = dac_ref.lock().unwrap();
                    let phase = q15_add(state.cur_phase, PHASE_OFFSET);
                    let sin_value: i32 = fast_sin(phase);
                    let amplitude = {
                        let amp =
                            q15_to_float(q15_sub(state.auto_offset_max, state.auto_offset_min))
                                * 0.5;
                        (amp * 2048.0).min(2048.0)
                    };
                    let dac_value =
                        (q15_to_float(sin_value) * amplitude + 2048.0).clamp(0.0, 4095.0) as u32;
                    dac.write(dac_value);
                }
            }
        });
    }
}

#[embassy_executor::task]
async fn display_task(_spawner: Spawner) {
    let display = Display::new();
    let mut cur_phase: i32 = 0;
    let mut omega: i32 = 0;
    let mut auto_offset_min: i32 = 0;
    let mut auto_offset_max: i32 = 0;
    let mut sogi_s1: i32 = 0;
    let mut sogi_s2: i32 = 0;
    let mut last_error: i32 = 0;
    let mut duration_ns: u32 = 0;
    let mut lock: bool = false;

    let _ = Timer::after(Duration::from_millis(100));
    display.set_backlight(1);

    loop {
        critical_section::with(|cs| {
            let state_ref = SOGI_STATE_REF.borrow(cs).get().unwrap();
            let state = state_ref.lock().unwrap();
            cur_phase = state.cur_phase;
            omega = state.omega;
            auto_offset_min = state.auto_offset_min;
            auto_offset_max = state.auto_offset_max;
            sogi_s1 = state.sogi_s1;
            sogi_s2 = state.sogi_s2;
            last_error = state.last_error;
            duration_ns = state.duration_ns;
            lock = state.is_lock((30e-3 * Q15_SCALE) as i32);
        });
        let theta_scaled = q15_to_float(q15_mul(cur_phase, THETA_SCALE));
        let freq = q15_to_float(omega) / TWO_PI;

        display.clear();
        let msg = format!(
            "sPLL:{}   F:{:<2}Hz   T: {:.2} uS",
            lock as u8,
            freq as u8,
            (duration_ns as f32 / 1e3)
        );
        display.write(msg.as_bytes());

        log::info!(
                ">OFFSET_MAX:{:.3}, OFFSET_MIN:{:.3}, OMEGA:{:.3}, THETA:{:.3}, S1:{:.3}, S2:{:.3}, ERR:{:.3}, FREQ:{:.3}, D_TIME:{}\n\0",
                auto_offset_min,
                auto_offset_max,
                omega,
                theta_scaled,
                sogi_s1,
                sogi_s2,
                last_error,
                freq,
                (duration_ns as f32 / 1e3)
            );

        let _ = Timer::after(Duration::from_millis(100));
    }
}

static EXECUTOR_MAIN: StaticCell<Executor> = StaticCell::new();

#[no_mangle]
extern "C" fn rust_main() {
    let _ = usage::set_logger();

    let sogi_state = SOGI_STATE.init(Mutex::new(SogiPllState::new()));
    critical_section::with(|cs| {
        SOGI_STATE_REF
            .borrow(cs)
            .set(sogi_state)
            .expect("SOGI_STATE_REF zaten başlatılmış!");
    });

    let dac_state = DAC_STATE.init(Mutex::new(Dac::new()));
    critical_section::with(|cs| {
        DAC_STATE_REF
            .borrow(cs)
            .set(dac_state)
            .expect("DAC_STATE_REF zaten başlatılmış!");
    });

    let mut adc = Adc::new();
    adc.read_async_isr(
        Duration::from_micros((1e6 / SAMPLE_FREQ) as u64).into(),
        Some(adc_callback),
    );

    let executor = EXECUTOR_MAIN.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(display_task(spawner)).unwrap();
    })
}
