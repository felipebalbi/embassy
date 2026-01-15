#![no_std]
#![no_main]

use embassy_executor::Spawner;
use hal::reset_reason::reset_reason;
use hal::tamper::{Config, ResetOnTamper, Tdet};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("Tamper detection example");

    for reason in reset_reason().into_iter() {
        defmt::info!("Reset Reason: '{}'", reason);
    }

    let mut config = Config::default();
    config.reset_on_tamper = ResetOnTamper::Yes;
    let _tamper = Tdet::new_no_ext_tamper(p.TDET0, config);

    embassy_time::Timer::after_secs(5).await;

    // force tamper by writing to TSR register
    let regs = hal::pac::TDET0;
    regs.tsr().write(|w| w.0 = 0);
}
