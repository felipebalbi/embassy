//! WUU DeepSleep wake on FRDM-MCXA577: SW2 is P3_17, WUU input 26, falling edge.
//!
//! ```text
//! cargo run --manifest-path examples/mcxa5xx/Cargo.toml --release --target thumbv8m.main-none-eabihf --no-default-features --features=executor-platform --bin wuu-wake-deepsleep
//! ```
//!
//! Blue: armed and waiting. Green: one wake completed. Red: timeout, failure,
//! or eight completed waits. Each wait has a 30 s OSTIMER timeout.
//! FRO16K supplies the WUU detector. `AllPowerModes` is required because
//! DeepSleep is not a low-leakage mode. The pad remains GPIO input with pull-up.
//! DeepSleep drops SWD. Recover by holding ISP, tapping RESET, then releasing ISP.
//! This MCXA5 example has not been exercised on hardware.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxa as hal;
use embassy_mcxa::clocks::PoweredClock;
use embassy_mcxa::clocks::config::{
    CoreSleep, Div8, FircConfig, FircFreqSel, FlashSleep, Fro16KConfig, MainClockConfig, MainClockSource,
    VddDriveStrength, VddLevel,
};
use embassy_mcxa::gpio::Pull;
use embassy_mcxa::wuu::{self, AllPowerModes, Edge, PinWake, Wuu};
use embassy_mcxa::{bind_interrupts, peripherals};
use embassy_time::{Duration, TimeoutError, with_timeout};
use hal::gpio::{DriveStrength, Level, Output, SlewRate};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    WUU0 => wuu::InterruptHandler<peripherals::WUU0>;
});

const WAKE_BUDGET: u32 = 8;
const WAIT_LIMIT: Duration = Duration::from_secs(30);

// Active-low RGB; each call selects one colour.
fn set_leds(red: &mut Output<'_>, green: &mut Output<'_>, blue: &mut Output<'_>, r: Level, g: Level, b: Level) {
    red.set_level(r);
    green.set_level(g);
    blue.set_level(b);
}

// The stock executor does not enter the configured low-power mode.
#[cfg(not(feature = "executor-platform"))]
compile_error!(
    "wuu-wake-deepsleep requires --no-default-features --features=executor-platform. Under the stock \
     embassy executor CoreSleep only warns and never sleeps, so a build without this feature \
     is a silent no-op that demonstrates nothing."
);

#[cfg_attr(
    feature = "executor-platform",
    embassy_executor::main(executor = "embassy_mcxa::executor::Executor", entry = "cortex_m_rt::entry")
)]
async fn main(_spawner: Spawner) {
    // Leave an attach window before changing power configuration.
    defmt::info!("Pre-power delay!");
    cortex_m::asm::delay(48_000_000);
    defmt::info!("Pre-power delay complete!");

    let mut cfg = hal::config::Config::default();

    let mut fcfg = FircConfig::default();
    fcfg.frequency = FircFreqSel::Mhz192;
    fcfg.power = PoweredClock::NormalEnabledDeepSleepDisabled;
    fcfg.fro_hf_enabled = true;
    fcfg.clk_hf_fundamental_enabled = false;
    fcfg.fro_hf_div = Div8::from_divisor(192);
    cfg.clock_cfg.firc = Some(fcfg);

    // Retain SIRC for OSTIMER and FRO16K for the WUU.
    cfg.clock_cfg.sirc.fro_12m_enabled = true;
    cfg.clock_cfg.sirc.fro_lf_div = None;
    cfg.clock_cfg.sirc.power = PoweredClock::AlwaysEnabled;
    cfg.clock_cfg.fro16k = Some(Fro16KConfig::default());

    cfg.clock_cfg.sosc = None;
    cfg.clock_cfg.spll = None;

    cfg.clock_cfg.main_clock = MainClockConfig {
        source: MainClockSource::FircHfRoot,
        power: PoweredClock::NormalEnabledDeepSleepDisabled,
        ahb_clk_div: Div8::no_div(),
    };

    cfg.clock_cfg.vdd_power.active_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.active_mode.drive = VddDriveStrength::Normal;
    cfg.clock_cfg.vdd_power.low_power_mode.level = VddLevel::MidDriveMode;
    cfg.clock_cfg.vdd_power.low_power_mode.drive = VddDriveStrength::Low { enable_bandgap: false };
    cfg.clock_cfg.vdd_power.core_sleep = CoreSleep::DeepSleep;
    cfg.clock_cfg.vdd_power.flash_sleep = FlashSleep::FlashDoze;

    let p = hal::init(cfg);

    let mut red = Output::new(p.P2_14, Level::High, DriveStrength::Normal, SlewRate::Slow);
    let mut green = Output::new(p.P2_22, Level::High, DriveStrength::Normal, SlewRate::Slow);
    let mut blue = Output::new(p.P2_23, Level::High, DriveStrength::Normal, SlewRate::Slow);

    let button = p.P3_17;

    let ctrl = match Wuu::new(p.WUU0, Irqs) {
        Ok(ctrl) => ctrl,
        Err(failure) => {
            defmt::error!("WUU construction refused: {}", failure.error);
            set_leds(&mut red, &mut green, &mut blue, Level::Low, Level::High, Level::High);
            return;
        }
    };

    let wake_cfg: PinWake<wuu::Interrupt, AllPowerModes> = PinWake::interrupt(Edge::Falling);

    let mut wake = match ctrl.arm_pin(button, Pull::Up, wake_cfg) {
        Ok(handle) => handle,
        Err(failure) => {
            defmt::error!("WUU arm refused: {}", failure.error);
            set_leds(&mut red, &mut green, &mut blue, Level::Low, Level::High, Level::High);
            return;
        }
    };

    defmt::info!("Armed P3_17 as WUU input 26, falling edge, all power modes. Waiting.");

    for iteration in 0..WAKE_BUDGET {
        set_leds(&mut red, &mut green, &mut blue, Level::High, Level::High, Level::Low);

        // Timeout cancels the waiter without consuming a retained WUU event.
        match with_timeout(WAIT_LIMIT, wake.wait_for_wake()).await {
            Ok(()) => {
                set_leds(&mut red, &mut green, &mut blue, Level::High, Level::Low, Level::High);
                defmt::info!("Woke from deep sleep, iteration {}", iteration);
                cortex_m::asm::delay(48_000_000 / 4);
            }
            Err(TimeoutError) => {
                defmt::error!("No wake within the wait limit; terminating.");
                set_leds(&mut red, &mut green, &mut blue, Level::Low, Level::High, Level::High);
                return;
            }
        }
    }

    // Dropping wake disarms the input.
    set_leds(&mut red, &mut green, &mut blue, Level::Low, Level::High, Level::High);
    defmt::warn!("Wake budget exhausted; terminating.");
}
