#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::bind_interrupts;
use hal::clocks::config::Div8;
use hal::config::Config;
use hal::freqme::{self, Freqme, InterruptHandler};
use hal::peripherals::FREQME0;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        FREQME0 => InterruptHandler<FREQME0>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    info!("Frequency Measurement example");

    let config = freqme::Config::default();
    let mut freqme = Freqme::new(p.FREQME0, Irqs, config).unwrap();

    loop {
        let f = freqme.measure().await.unwrap();
        defmt::info!("Frequency {} Hz", f);
        Timer::after_secs(1).await;
    }
}
