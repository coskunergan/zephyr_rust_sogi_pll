// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

extern "C" {
    fn display_init() -> i32;
    fn display_write(data: *const u8, len: u16) -> i32;
    fn display_clear() -> i32;
    fn display_set_cursor(state: bool) -> i32;
    fn display_set_backlight(state: u8) -> i32;
}

pub struct Display {
    _private: (),
}

impl Display {
    pub fn new() -> Self {
        let ret = unsafe { display_init() };
        if ret != 0 {
            //panic!("Failed to initialize display: error {}", ret);
        }

        Display { _private: () }
    }

    pub fn write(&self, data: &[u8]) {
        let ret = unsafe { display_write(data.as_ptr(), data.len() as u16) };
        if ret != 0 {
            //panic!("Failed to write to display: error {}", ret);
        }
    }
    #[allow(dead_code)]
    pub fn clear(&self) {
        let ret = unsafe { display_clear() };
        if ret != 0 {
            //panic!("Failed to clear display: error {}", ret);
        }
    }
    #[allow(dead_code)]
    pub fn set_cursor(&self, state: bool) {
        let ret = unsafe { display_set_cursor(state) };
        if ret != 0 {
            //panic!("Failed to set display cursor: error {}", ret);
        }
    }
    #[allow(dead_code)]
    pub fn set_backlight(&self, state: u8) {
        let ret = unsafe { display_set_backlight(state) };
        if ret != 0 {
            //panic!("Failed to set display backlight: error {}", ret);
        }
    }
}
