// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#![no_std]
//#![allow(warnings)]

extern crate alloc;

use embassy_time::{Duration, Timer};

use alloc::format;

#[cfg(feature = "executor-thread")]
use embassy_executor::Executor;

#[cfg(feature = "executor-zephyr")]
use zephyr::embassy::Executor;

use zephyr::{
    sync::{Arc, Mutex},
};

use core::{sync::atomic::AtomicBool, sync::atomic::AtomicI32, sync::atomic::Ordering};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

use adc_io::Adc;
use dac_io::Dac;
use display_io::Display;

mod adc_io;
mod dac_io;
mod display_io;

static EXECUTOR_MAIN: StaticCell<Executor> = StaticCell::new();
pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static DISPLAY: spin::Once<Display> = spin::Once::new();
static DAC: spin::Once<Dac> = spin::Once::new();

#[no_mangle]
extern "C" fn rust_main() {
    unsafe {
        zephyr::set_logger().unwrap();
    }

    let executor = EXECUTOR_MAIN.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(main(spawner)).unwrap();
    })
}

#[embassy_executor::task]
async fn main(spawner: Spawner) {

    let display = Display::new();
    DISPLAY.call_once(|| display);

    let dac = Dac::new();
    DAC.call_once(|| dac);

    let mut adc = Adc::new();
    adc.read_async(
        core::time::Duration::from_millis(500),
        Some(|idx, value| {
            zephyr::printk!("ADC Channel {}: {}\n", idx, value);
            if idx == 0 {
                if let Some(dac) = DAC.get() {
                    dac.write(value as i32);
                }

                if let Some(display) = DISPLAY.get() {
                    display.clear();
                    let msg = format!("ADC {}: {}", idx, value);
                    display.write(msg.as_bytes());
                }
            }
        }),
    );

    Timer::after(Duration::from_millis(30)).await;

    if let Some(display) = DISPLAY.get() {    
        display.set_backlight(1);
    }

    loop {    
        DISPLAY_SIGNAL.wait().await;
    }
}
