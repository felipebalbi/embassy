//! Blinky, but with the clock tree declared via `validated_clocks!`.
//!
//! The SPLL is sourced from SIRC and drives `main_clk`. The whole tree is
//! declared at module scope and resolved and checked at COMPILE time: if the
//! configuration were not legal, this file would fail to build at the
//! `validated_clocks!` invocation rather than faulting at runtime.
//!
//! The resulting [`ValidatedClocksConfig`] token is handed to
//! `hal::init_validated()`, so the configuration value that was checked is the
//! value passed to clock initialisation. That binds the input configuration; it
//! does not prove the runtime or the hardware reaches the asserted tree.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxa as hal;
use embassy_mcxa::clocks::PoweredClock;
use embassy_mcxa::clocks::config::{ClocksConfig, MainClockSource, SpllConfig, SpllMode, SpllSource};
use embassy_time::Timer;
use hal::gpio::{DriveStrength, Level, Output, SlewRate};
use panic_probe as _;

hal::validated_clocks! {
    /// Clock tree for this example: SPLL at 12 MHz, sourced from SIRC, driving `main_clk`.
    pub mod board_clocks {
        clock_config: {
            let mut c = ClocksConfig::new();
            c.spll = Some(SpllConfig {
                source: SpllSource::Sirc,
                // 12MHz
                // 12 x 32 => 384MHz
                // 384 / (16 x 2) => 12.0MHz
                mode: SpllMode::Mode1b {
                    m_mult: 32,
                    p_div: 16,
                    bypass_p2_div: false,
                },
                power: PoweredClock::NormalEnabledDeepSleepDisabled,
                pll1_clk_div: None,
            });
            c.main_clock.source = MainClockSource::SPll1;
            c
        };
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init_validated(hal::config::Config::default(), board_clocks::VALIDATED);

    defmt::info!("Blink example");

    let mut red = Output::new(p.P2_14, Level::High, DriveStrength::Normal, SlewRate::Fast);
    let mut green = Output::new(p.P2_22, Level::High, DriveStrength::Normal, SlewRate::Fast);
    let mut blue = Output::new(p.P2_23, Level::High, DriveStrength::Normal, SlewRate::Fast);

    loop {
        defmt::info!("Toggle LEDs");

        red.toggle();
        Timer::after_millis(250).await;

        red.toggle();
        green.toggle();
        Timer::after_millis(250).await;

        green.toggle();
        blue.toggle();
        Timer::after_millis(250).await;
        blue.toggle();

        Timer::after_millis(250).await;
    }
}
