use defmt::info;
use edrv_ltr390uv::{LTR390UV};
use embassy_nrf::{
    bind_interrupts,
    peripherals::{P0_04, P0_05, TWISPI0},
    twim::{self, Twim},
};
use embassy_time::{Delay, Timer};

#[embassy_executor::task]
pub async fn measure_light_task(twi: TWISPI0, sda: P0_04, scl: P0_05) {
    bind_interrupts!(struct Irqs {
        TWISPI0 => twim::InterruptHandler<TWISPI0>;
    });
    let i2c = Twim::new(twi, Irqs, sda, scl, Default::default());

    let mut ltr390 = LTR390UV::new_primary(i2c);
    Timer::after_millis(30).await;
    let config = edrv_ltr390uv::Config {
        resolution: edrv_ltr390uv::Resolution::Bit20,
        rate: edrv_ltr390uv::MeasurementRate::Ms2000Alt,
        gain: edrv_ltr390uv::GainRange::Gain18,
    };
    ltr390.init(config).await.unwrap();

    loop {
        info!("Sensor serial number: {}", ltr390.read_uvs_data(Delay).await.unwrap());
        Timer::after_millis(500).await;
    }
}
