// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

extern "C" {
    fn dac_init() -> i32;
    fn dac_write(value: u32) -> i32;
}

pub struct Dac {
    _private: (),
}

impl Dac {
    pub fn new() -> Self {
        let ret = unsafe { dac_init() };
        if ret != 0 {
            panic!("Failed to initialize DAC: error {}", ret);
        }

        Dac { _private: () }
    }

    pub fn write(&self, value: u32) {
        let ret = unsafe { dac_write(value) };
        if ret != 0 {
            panic!("Failed to write to DAC: error {}", ret);
        }
    }
}
