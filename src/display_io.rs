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
        unsafe { display_init(); };        
        Display { _private: () }
    }

    pub fn write(&self, data: &[u8])  -> i32{
        unsafe { display_write(data.as_ptr(), data.len() as u16) }
    }
    
    #[allow(dead_code)]
    pub fn clear(&self) -> i32{
        unsafe { display_clear() }    
    }
    #[allow(dead_code)]
    pub fn set_cursor(&self, state: bool) -> i32{
        unsafe { display_set_cursor(state) }        
    }
    #[allow(dead_code)]
    pub fn set_backlight(&self, state: u8) -> i32{
        unsafe { display_set_backlight(state) }
    }
}
