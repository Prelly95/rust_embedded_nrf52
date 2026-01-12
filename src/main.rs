#![no_std]
#![no_main]
use core::ptr::write_volatile;
use cortex_m_rt::entry;
use cortex_m::asm::nop;
use panic_halt as _;


#[entry]
fn main() -> ! {
    let gpio_pin17_config: *mut u32 = 0x50_000_744 as *mut u32;
    let dir_config_pos: u32 = 0;
    let pin17_config: u32 = 1 << dir_config_pos;
    unsafe {
        write_volatile(gpio_pin17_config, pin17_config);
    }
    let gpio_out_addr: *mut u32 = 0x50_000_504 as *mut u32;
    let pin17_out_pos: u32 = 17;
    let mut pin_state: bool = true;
    loop {
        unsafe {
            write_volatile(gpio_out_addr, (pin_state as u32) << pin17_out_pos);
        }
        for _ in 0..40_00_000 {
            nop();
        }
        pin_state = !pin_state
    }
}
