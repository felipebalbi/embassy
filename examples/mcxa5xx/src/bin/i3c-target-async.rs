#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::bind_interrupts;
use embassy_mcxa::clocks::PoweredClock;
use embassy_mcxa::clocks::config::{
    CoreSleep, Div8, FircConfig, FircFreqSel, FlashSleep, MainClockConfig, MainClockSource, VddDriveStrength, VddLevel,
};
use embassy_mcxa::clocks::periph_helpers::{Div4, I3cClockSel};
use embassy_mcxa::i3c::target;
use embassy_mcxa::peripherals::I3C0;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        I3C0 => target::InterruptHandler<I3C0>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut cfg = hal::config::Config::default();

    // Enable 192MHz clock source
    let mut fcfg = FircConfig::default();
    fcfg.frequency = FircFreqSel::Mhz192;
    fcfg.power = PoweredClock::NormalEnabledDeepSleepDisabled;
    fcfg.fro_hf_enabled = true;
    fcfg.clk_hf_fundamental_enabled = false;
    fcfg.fro_hf_div = Some(const { Div8::from_divisor(4).unwrap() });
    cfg.clock_cfg.firc = Some(fcfg);

    // Enable 12M osc
    cfg.clock_cfg.sirc.fro_12m_enabled = true;
    cfg.clock_cfg.sirc.fro_lf_div = Some(Div8::no_div());
    cfg.clock_cfg.sirc.power = PoweredClock::AlwaysEnabled;

    // Disable 16K osc
    cfg.clock_cfg.fro16k = None;

    // Disable external osc
    cfg.clock_cfg.sosc = None;

    // Disable PLL
    cfg.clock_cfg.spll = None;

    // Feed core from 192M osc
    cfg.clock_cfg.main_clock = MainClockConfig {
        source: MainClockSource::FircHfRoot,
        power: PoweredClock::NormalEnabledDeepSleepDisabled,
        ahb_clk_div: Div8::no_div(),
    };

    // We don't sleep, set relatively high power
    cfg.clock_cfg.vdd_power.active_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.low_power_mode.level = VddLevel::MidDriveMode;
    cfg.clock_cfg.vdd_power.active_mode.drive = VddDriveStrength::Normal;
    cfg.clock_cfg.vdd_power.low_power_mode.drive = VddDriveStrength::Low { enable_bandgap: false };

    // Set "never sleep" mode
    cfg.clock_cfg.vdd_power.core_sleep = CoreSleep::WfeUngated;

    // Set flash doze, allowing internal flash clocks to be gated on sleep
    cfg.clock_cfg.vdd_power.flash_sleep = FlashSleep::FlashDoze;

    let p = hal::init(cfg);

    defmt::info!("I3C target example");

    let mut config = target::Config::default();
    config.clock_config.source = I3cClockSel::FroHfDiv;
    config.clock_config.div = Div4::from_divisor(2).unwrap();
    config.address = Some(0x2a);

    let mut target = target::I3c::new_async(p.I3C0, p.P0_21, p.P0_20, Irqs, config).unwrap();
    let mut rx_buf = [0u8; 32];
    let tx_buf = [0x2au8; 32];

    loop {
        let rx_count = target.async_respond_to_write(&mut rx_buf).await.unwrap();
        defmt::info!("T [W]: {:02x}", rx_buf[..rx_count]);

        match target.async_respond_to_read(&tx_buf).await.unwrap() {
            target::ReadStatus::EarlyStop(count) => {
                defmt::info!("T [W]: {:02x}", rx_buf[..rx_count]);
                defmt::info!("T [Re] ({}): {:02x}", count, tx_buf[..count]);
            }
            target::ReadStatus::Complete(count) => {
                defmt::info!("T [W]: {:02x}", rx_buf[..rx_count]);
                defmt::info!("T [Rc] ({}): {:02x}", count, tx_buf[..count]);
            }
            target::ReadStatus::Incomplete(count) => {
                defmt::info!("T [W]: {:02x}", rx_buf[..rx_count]);
                defmt::info!("T [Ri] ({}): {:02x}", count, tx_buf[..count]);
            }
        }
    }
}
