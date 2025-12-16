#![no_std]
#![no_main]
use core::ptr::write_volatile;
use cortex_m_rt::entry;
use cortex_m::asm::nop;
use panic_halt as _;


#[entry]
fn main() -> ! {
    let gpio_pin30_config: *mut u32 = 0x50_000_778 as *mut u32;
    let dir_config_pos: u32 = 0;
    let pin30_config: u32 = 1 << dir_config_pos;
    unsafe {
        write_volatile(gpio_pin30_config, pin30_config);
    }
    let gpio_out_addr: *mut u32 = 0x50_000_504 as *mut u32;
    let pin30_out_pos: u32 = 30;
    let mut pin_state: bool = true;
    loop {
        unsafe {
            write_volatile(gpio_out_addr, (pin_state as u32) << pin30_out_pos);
        }
        for _ in 0..400_000 {
            nop();
        }
        pin_state = !pin_state
    }
}
