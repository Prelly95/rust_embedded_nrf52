#![no_std]
#![no_main]

mod sensors;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;
use sensors::measure_light_task;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting...");
    let p = embassy_nrf::init(Default::default());
    let scl = p.P0_05;
    let sda = p.P0_04;

    spawner.must_spawn(measure_light_task(p.TWISPI0, sda, scl));
}
