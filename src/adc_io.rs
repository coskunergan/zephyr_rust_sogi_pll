// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use core::ffi::c_void;
use core::time::Duration;
use zephyr::raw::device;

#[repr(C)]
pub struct adc_dt_spec {
    pub dev: *const device,
    pub channel_id: u32,
    pub resolution: u8,
    pub oversampling: u8,
}

extern "C" {
    fn get_adc_channels() -> *const adc_dt_spec;
    fn get_adc_channels_len() -> usize;
    fn adc_new() -> *mut c_void;
    fn adc_async_read(
        adc: *mut c_void,
        interval_us: u32,
        handler: Option<extern "C" fn(*mut c_void, usize, i16)>,
        user_data: *mut c_void,
    ) -> i32;
    fn adc_async_read_isr(
        adc: *mut c_void,
        interval_us: u32,
        handler: Option<extern "C" fn(*mut c_void, usize, i16)>,
        user_data: *mut c_void,
    ) -> i32;
    fn adc_cancel_read(adc: *mut c_void);
    fn adc_get_voltage(adc: *mut c_void, idx: usize) -> i32;
    fn adc_get_value(adc: *mut c_void, idx: usize) -> i32;
    fn adc_free(adc: *mut c_void);
}

pub struct Adc {
    adc_ptr: *mut c_void,
    channel_count: usize,
    done_cb: Option<fn(usize, i16)>,
    done_cb_isr: Option<fn(usize, i16)>,
}

impl Adc {
    pub fn new() -> Self {
        let adc_ptr = unsafe { adc_new() };
        if adc_ptr.is_null() {
            panic!("Failed to create ADC object");
        }

        let channel_count = unsafe { get_adc_channels_len() };
        if channel_count == 0 {
            unsafe { adc_free(adc_ptr) };
            panic!("adc_channels_len is 0");
        }

        let channels_ptr = unsafe { get_adc_channels() };
        if channels_ptr.is_null() {
            unsafe { adc_free(adc_ptr) };
            panic!("adc_channels is null");
        }

        Adc {
            adc_ptr,
            channel_count,
            done_cb: None,
            done_cb_isr: None,
        }
    }
    #[allow(dead_code)]
    pub fn read_async(&mut self, interval: Duration, handler: Option<fn(usize, i16)>) {
        let interval_us = interval.as_micros() as u32;
        self.done_cb = handler;

        let res = unsafe {
            adc_async_read(
                self.adc_ptr,
                interval_us,
                Some(Self::callback_wrapper),
                self as *mut Self as *mut c_void,
            )
        };
        if res != 0 {
            panic!("Failed to start async ADC read");
        }
    }
    #[allow(dead_code)]
    pub fn read_async_isr(&mut self, interval: Duration, handler: Option<fn(usize, i16)>) {
        let interval_us = interval.as_micros() as u32;
        self.done_cb_isr = handler;

        let res = unsafe {
            adc_async_read_isr(
                self.adc_ptr,
                interval_us,
                Some(Self::callback_wrapper),
                self as *mut Self as *mut c_void,
            )
        };
        if res != 0 {
            panic!("Failed to start async ADC read in ISR");
        }
    }
    #[allow(dead_code)]
    pub fn cancel_read(&mut self) {
        unsafe { adc_cancel_read(self.adc_ptr) };
    }
    #[allow(dead_code)]
    pub fn get_voltage(&self, idx: usize) -> i32 {
        if idx >= self.channel_count {
            panic!("Index out of bounds: {}", idx);
        }
        unsafe { adc_get_voltage(self.adc_ptr, idx) }
    }
    #[allow(dead_code)]
    pub fn get_value(&self, idx: usize) -> i32 {
        if idx >= self.channel_count {
            panic!("Index out of bounds: {}", idx);
        }
        unsafe { adc_get_value(self.adc_ptr, idx) }
    }

    extern "C" fn callback_wrapper(user_data: *mut c_void, idx: usize, val: i16) {
        let adc = unsafe { &mut *(user_data as *mut Self) };
        if let Some(cb) = adc.done_cb {
            cb(idx, val);
        }
        if let Some(cb) = adc.done_cb_isr {
            cb(idx, val);
        }
    }
}

impl Drop for Adc {
    fn drop(&mut self) {
        unsafe { adc_free(self.adc_ptr) };
    }
}
