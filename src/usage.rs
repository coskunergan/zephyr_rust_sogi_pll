use zephyr::raw::k_cycle_get_32;

// Performans ölçümü için statik değişken
static mut LAST_CYCLES: u32 = 0;

// Zephyr döngü sayacını güvenli şekilde oku
pub fn get_cycle_count() -> u32 {
    unsafe { k_cycle_get_32() }
}

// LAST_CYCLES’a yazma
pub fn set_last_cycles(value: u32) {
    unsafe {
        LAST_CYCLES = value;
    }
}

// LAST_CYCLES’tan okuma
pub fn get_last_cycles() -> u32 {
    unsafe { LAST_CYCLES }
}

// Zephyr logger’ı güvenli şekilde ayarla
pub fn set_logger_safe() -> Result<(), &'static str> {
    unsafe { zephyr::set_logger() }.map_err(|_| "Logger ayarı başarısız")
}
