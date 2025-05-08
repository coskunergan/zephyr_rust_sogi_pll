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

// SOGI-PLL durum yapısı
struct SogiPllState {
    v_alpha: i16,  // Q15
    v_beta: i16,   // Q15
    omega: i16,    // Q15 (2π * 50 Hz)
    theta: u16,    // 0-65535 (2π)
    integral: i32, // Q15
}

impl SogiPllState {
    const fn new() -> Self {
        SogiPllState {
            v_alpha: 0,
            v_beta: 0,
            omega: 31416, // 2π * 50 Hz, Q15
            theta: 0,
            integral: 0,
        }
    }
}

// Statik SOGI-PLL durumu (Mutex ile Sync)
static SOGI_STATE: StaticCell<Mutex<SogiPllState>> = StaticCell::new();
static SOGI_STATE_REF: Once<&'static Mutex<SogiPllState>> = Once::new();
static DAC: spin::Once<Dac> = spin::Once::new();

// Sinüs lookup table (256 eleman, Q15)
const SIN_LUT: [i16; 256] = [
    0, 804, 1608, 2411, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8740, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787,
    21403, 22005, 22594, 23170, 23732, 24279, 24812, 25330, 25832, 26319, 26790, 27245, 27683,
    28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113, 31356, 31580, 31785,
    31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678,
    32609, 32521, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571, 30273,
    29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25330, 24812,
    24279, 23732, 23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846,
    16151, 15446, 14732, 14010, 13279, 12539, 11793, 11039, 10278, 9512, 8740, 7962, 7179, 6393,
    5602, 4808, 4011, 3212, 2411, 1608, 804, 0, -804, -1608, -2411, -3212, -4011, -4808, -5602,
    -6393, -7179, -7962, -8740, -9512, -10278, -11039, -11793, -12539, -13279, -14010, -14732,
    -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23732, -24279, -24812, -25330, -25832, -26319, -26790, -27245, -27683, -28105, -28510,
    -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757, -32767, -32757, -32728, -32678,
    -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832,
    -25330, -24812, -24279, -23732, -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279, -12539, -11793, -11039, -10278,
    -9512, -8740, -7962, -7179, -6393, -5602, -4808, -4011, -3212, -2411, -1608, -804,
];

// SOGI bloğu (Q15)
fn sogi_process_q15(
    input: i16,
    omega: i16,
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
// fn approx_phase_error(v_alpha: i16, v_beta: i16, _theta: u16) -> i16 {
//     if v_alpha == 0 {
//         0
//     } else {
//         TryInto::<i16>::try_into(((v_beta as i32) << 15) / v_alpha as i32).unwrap()
//     }
// }

fn approx_phase_error(v_alpha: i16, v_beta: i16, _theta: u16) -> i16 {
    if v_alpha == 0 {
        0
    } else {
        match (((v_beta as i32) << 15) / v_alpha as i32).try_into() {
            Ok(val) => val,
            Err(_) => {                
                #[cfg(debug_assertions)]
                zephyr::printk!("approx_phase_error: i32 -> i16 donusum basarisiz oldu\n");
                0 
            }
        }
    }
}

// PI denetleyici (Q15)
fn pi_controller_q15(error: i16, kp: i16, ki: i16, ts: i16, state: &mut SogiPllState) -> i16 {
    state.integral += ((ki as i32 * error as i32) >> 15) * ts as i32;
    TryInto::<i16>::try_into(((kp as i32 * error as i32) >> 15) + (state.integral >> 15)).unwrap()
}

// ADC geri çağrı fonksiyonu
fn adc_callback(idx: usize, value: i16) {
    // Sabitler
    const K: i16 = 4634; // 1.414 * 32768 / 2^15
    const TS: i16 = 1638; // 500 µs (2 kHz), Q15
    const KP: i16 = 3277; // 0.1 * 32768
    const KI: i16 = 327; // 0.01 * 32768
    const NOMINAL_OMEGA: i16 = 31416; // 2π * 50 Hz, Q15

    if idx == 0 {
        // SOGI-PLL durumuna güvenli erişim
        if let Ok(mut state) = SOGI_STATE_REF.get().unwrap().lock() {
            // SOGI bloğu
            let (v_alpha, v_beta) = sogi_process_q15(value, state.omega, K, TS, &mut state);

            // Faz hatası
            let phase_error = approx_phase_error(v_alpha, v_beta, state.theta);

            // PI denetleyici
            let omega_delta = pi_controller_q15(phase_error, KP, KI, TS, &mut state);
            if (NOMINAL_OMEGA + omega_delta) < 32767 {
                state.omega = NOMINAL_OMEGA + omega_delta;    
            }                       

            // Faz açısını güncelle
            state.theta += ((state.omega as i32 * TS as i32) >> 15) as u16;

            if state.theta > 50000 {
                state.theta = 0;
            }

            //DAC çıkışı
            if let Some(dac) = DAC.get() {
                let output = SIN_LUT[((state.theta >> 8) as usize)] as u32;                
                dac.write(output % 4095);
            }

            // Hata ayıklama
            #[cfg(debug_assertions)]
            zephyr::printk!(
                "ADC: {}, v_alpha: {}, v_beta: {}, theta: {}\n",
                value,
                v_alpha,
                v_beta,
                state.theta
            );
        }
    }
}

// Ekran güncelleme görevi
#[embassy_executor::task]
async fn display_task() {
    let display = Display::new();
    DISPLAY.call_once(|| display);

    // SOGI-PLL durumunu başlat
    let sogi_state = SOGI_STATE.init(Mutex::new(SogiPllState::new()));
    SOGI_STATE_REF.call_once(|| sogi_state);

    loop {
        if let Ok(state) = SOGI_STATE_REF.get().unwrap().lock() {
            if let Some(display) = DISPLAY.get() {
                display.clear();
                let msg = format!("Alpha: {:<5}    Theta: {}", state.v_alpha, state.theta);
                display.write(msg.as_bytes());
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

static EXECUTOR_MAIN: StaticCell<Executor> = StaticCell::new();
pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static DISPLAY: spin::Once<Display> = spin::Once::new();

#[no_mangle]
extern "C" fn rust_main() {
    unsafe {
        zephyr::set_logger().unwrap();
    }

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

    //ADC’yi başlat
    let mut adc = Adc::new();
    adc.read_async(
        Duration::from_micros(50000).into(), // 2 kHz (500 µs)
        Some(adc_callback),
    );

    if let Some(display) = DISPLAY.get() {
        display.set_backlight(1);
    }

    loop {
        DISPLAY_SIGNAL.wait().await;
    }
}
